#!/usr/bin/env python3
"""
route_server.py — SWAGGER graph-based route planner for Nav2.

Ingests a GML waypoint graph produced by SWAGGER SLAM, finds the
shortest path between two poses via Dijkstra's algorithm with
composite edge scoring (distance + semantic + time), then densifies
the result to 5 cm resolution as a nav_msgs/Path.

Service   : ~/get_plan   (nav_msgs/srv/GetPlan)
Publisher : ~/path       (nav_msgs/msg/Path, transient-local for Rviz)
Subscriber: /goal_pose   (geometry_msgs/PoseStamped) — Rviz "2D Goal Pose"
             On receipt, robot's current TF pose is used as start and the
             path is planned + published automatically.
"""

import math
import os

import networkx as nx
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformException
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# GML parser — handles SWAGGER's repeated-key format
# ---------------------------------------------------------------------------

def _parse_gml(path: str) -> nx.Graph:
    """
    Parse a SWAGGER-style GML file into a networkx Graph.

    SWAGGER uses repeated keys for array values within a block:
        world  4.85   <- x
        world 32.55   <- y
        world  0.0    <- z
        pixel  9      <- pixel_x
        pixel 97      <- pixel_y

    Edge attributes carried: weight (float, metres), edge_type (str).
    Node attributes stored : x, y (world coords), node_type (str).
    """
    G = nx.Graph()

    in_graph = False
    in_node = False
    in_edge = False
    current: dict = {}
    world_vals: list[float] = []

    with open(path) as fh:
        for raw in fh:
            line = raw.strip()

            if line == "graph [":
                in_graph = True
                continue

            if not in_graph:
                continue

            if line == "node [":
                in_node = True
                current = {}
                world_vals = []
                continue

            if line == "edge [":
                in_edge = True
                current = {}
                continue

            if line == "]":
                if in_node:
                    in_node = False
                    nid = current.get("id")
                    if nid is not None and len(world_vals) >= 2:
                        G.add_node(
                            nid,
                            x=world_vals[0],
                            y=world_vals[1],
                            node_type=current.get("node_type", "normal"),
                        )
                elif in_edge:
                    in_edge = False
                    src = current.get("source")
                    tgt = current.get("target")
                    if src is not None and tgt is not None:
                        G.add_edge(
                            src, tgt,
                            weight=current.get("weight", 1.0),
                            edge_type=current.get("edge_type", "skeleton"),
                        )
                else:
                    in_graph = False
                continue

            # key–value line inside a block
            parts = line.split(None, 1)
            if len(parts) != 2:
                continue
            key, raw_val = parts

            if in_node:
                if key == "world":
                    world_vals.append(float(raw_val))
                elif key == "id":
                    current["id"] = int(raw_val)
                elif key == "node_type":
                    current["node_type"] = raw_val.strip('"')

            elif in_edge:
                if key == "source":
                    current["source"] = int(raw_val)
                elif key == "target":
                    current["target"] = int(raw_val)
                elif key == "weight":
                    current["weight"] = float(raw_val)
                elif key == "edge_type":
                    current["edge_type"] = raw_val.strip('"')

    return G


# ---------------------------------------------------------------------------
# Map-origin loader
# ---------------------------------------------------------------------------

def _load_map_origin(map_yaml_path: str) -> tuple[float, float]:
    """
    Read the origin [x, y] from a ROS2 map YAML file.
    The origin is the position of the bottom-left pixel of the PGM image
    in the ROS map frame, which equals the SLAM world frame.

    SWAGGER outputs graph world coords in the SLAM frame.
    AMCL reports poses in the ROS map frame.
    These frames are identical IF the map is generated directly from SLAM —
    the origin field just tells us where pixel (0,0) sits, not a frame offset.

    However if SWAGGER was run against the SLAM map image using its own
    coordinate convention (pixel * resolution, top-left origin) the world
    coords will be offset from the ROS map frame by the map origin.
    Applying the origin here corrects this.
    """
    with open(map_yaml_path) as f:
        data = yaml.safe_load(f)
    origin = data.get("origin", [0.0, 0.0, 0.0])
    return float(origin[0]), float(origin[1])


# ---------------------------------------------------------------------------
# Edge scorer
# ---------------------------------------------------------------------------

def _edge_cost(
    G: nx.Graph,
    u: int,
    v: int,
    w_distance: float,
    w_semantic: float,
    w_time: float,
    robot_speed: float,
    skeleton_only: bool,
    delaunay_multiplier: float,
) -> float:
    """
    Composite edge cost used by Dijkstra:

        cost = w_distance * geo_dist * dist_scale
             + w_semantic * boundary_penalty
             + w_time     * (geo_dist / robot_speed)

    dist_scale = 1.0 for skeleton edges (medial-axis, guaranteed obstacle-free).
               = delaunay_multiplier for Delaunay edges (may cross walls/gaps).
    boundary_penalty: 0.5 per endpoint that is a boundary node.
    """
    data = G[u][v]
    geo_dist: float = data["weight"]  # metres (Euclidean)
    etype: str = data.get("edge_type", "skeleton")

    if skeleton_only and etype != "skeleton":
        return float("inf")

    dist_scale = 1.0 if etype == "skeleton" else delaunay_multiplier

    sem_cost = 0.0
    if G.nodes[u].get("node_type") == "boundary":
        sem_cost += 0.5
    if G.nodes[v].get("node_type") == "boundary":
        sem_cost += 0.5

    time_cost = (geo_dist / max(robot_speed, 1e-3)) if w_time > 0.0 else 0.0

    return (
        w_distance * geo_dist * dist_scale
        + w_semantic * sem_cost
        + w_time * time_cost
    )


# ---------------------------------------------------------------------------
# RouteServer
# ---------------------------------------------------------------------------

class RouteServer(Node):
    def __init__(self) -> None:
        super().__init__("route_server")

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter("graph_file", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("interpolation_resolution", 0.05)
        self.declare_parameter("w_distance", 1.0)
        self.declare_parameter("w_semantic", 0.5)
        self.declare_parameter("w_time", 0.0)
        self.declare_parameter("robot_speed", 0.5)
        self.declare_parameter("skeleton_only", False)
        self.declare_parameter("delaunay_multiplier", 8.0)

        graph_file = self.get_parameter("graph_file").value
        map_yaml   = self.get_parameter("map_yaml").value
        self.frame_id = self.get_parameter("frame_id").value
        self._base_frame = self.get_parameter("base_frame").value
        self.res = self.get_parameter("interpolation_resolution").value
        self.w_dist = self.get_parameter("w_distance").value
        self.w_sem = self.get_parameter("w_semantic").value
        self.w_time = self.get_parameter("w_time").value
        self.speed = self.get_parameter("robot_speed").value
        self.skel_only = self.get_parameter("skeleton_only").value
        self.delaunay_mul = self.get_parameter("delaunay_multiplier").value

        # ── load graph ───────────────────────────────────────────────────────
        if not graph_file or not os.path.isfile(graph_file):
            self.get_logger().fatal(f"graph_file not found: '{graph_file}'")
            raise FileNotFoundError(graph_file)

        self.get_logger().info(f"Loading graph: {graph_file}")
        self._G = _parse_gml(graph_file)
        n_nodes = self._G.number_of_nodes()
        n_edges = self._G.number_of_edges()

        # ── align SWAGGER world frame → ROS map frame via map origin ─────────
        # SWAGGER outputs node coords in the SLAM world frame.
        # The ROS map frame IS the SLAM world frame but AMCL poses are offset
        # by the map YAML origin (bottom-left pixel position in world coords).
        # Applying the origin here makes both coordinate systems consistent.
        self._map_origin = (0.0, 0.0)
        if map_yaml and os.path.isfile(map_yaml):
            ox, oy = _load_map_origin(map_yaml)
            self._map_origin = (ox, oy)
            for nid in self._G.nodes():
                self._G.nodes[nid]["x"] += ox
                self._G.nodes[nid]["y"] += oy
            self.get_logger().info(
                f"Map origin offset applied: ({ox:.4f}, {oy:.4f})"
            )
        else:
            self.get_logger().warn(
                "map_yaml not set — graph coords may not align with AMCL frame"
            )

        self.get_logger().info(
            f"Graph loaded — {n_nodes} nodes, {n_edges} edges  "
            f"(skeleton_only={self.skel_only})"
        )

        # ── fast nearest-node lookup (vectorised) ────────────────────────────
        node_ids = list(self._G.nodes())
        self._node_ids = np.array(node_ids, dtype=np.int32)
        self._node_xy = np.array(
            [[self._G.nodes[n]["x"], self._G.nodes[n]["y"]] for n in node_ids],
            dtype=np.float64,
        )

        # ── publishers (transient-local so Rviz catches them on subscribe) ────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub  = self.create_publisher(Path,        "~/path",  latched)
        self._graph_pub = self.create_publisher(MarkerArray, "~/graph", latched)

        # publish graph immediately so it's visible in Rviz on startup
        self._publish_graph_markers()

        # ── service ──────────────────────────────────────────────────────────
        self._srv = self.create_service(
            GetPlan, "~/get_plan", self._cb_get_plan
        )

        # ── TF listener (fallback robot pose) ───────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── AMCL pose cache (primary start-pose source) ──────────────────────
        self._amcl_pose: PoseWithCovarianceStamped | None = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            lambda msg: setattr(self, "_amcl_pose", msg),
            10,
        )

        # ── Rviz "2D Goal Pose" subscriber ───────────────────────────────────
        self._goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self._cb_goal_pose, 10
        )
        self.get_logger().info(
            "Route server ready  (~/get_plan | /goal_pose → ~/path)"
        )

    # ── graph visualisation ──────────────────────────────────────────────────

    def _publish_graph_markers(self) -> None:
        """
        Publish the SWAGGER graph as a MarkerArray for Rviz.
          ~/graph  — latched, so it persists across Rviz restarts.
        Nodes : green spheres  (boundary nodes in orange)
        Edges : blue lines (skeleton) / grey lines (delaunay)
        """
        stamp = self.get_clock().now().to_msg()
        forever = DurationMsg(sec=0, nanosec=0)  # never auto-delete

        # ── nodes ─────────────────────────────────────────────────────────────
        nm = Marker()
        nm.header = Header(frame_id=self.frame_id, stamp=stamp)
        nm.ns, nm.id = "swagger_nodes", 0
        nm.type, nm.action = Marker.SPHERE_LIST, Marker.ADD
        nm.scale.x = nm.scale.y = nm.scale.z = 0.25
        nm.lifetime = forever

        for nid in self._G.nodes():
            nd = self._G.nodes[nid]
            nm.points.append(Point(x=nd["x"], y=nd["y"], z=0.15))
            if nd.get("node_type") == "boundary":
                nm.colors.append(ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0))
            else:
                nm.colors.append(ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0))

        # ── skeleton edges ────────────────────────────────────────────────────
        se = Marker()
        se.header = Header(frame_id=self.frame_id, stamp=stamp)
        se.ns, se.id = "swagger_skeleton_edges", 1
        se.type, se.action = Marker.LINE_LIST, Marker.ADD
        se.scale.x = 0.06
        se.color = ColorRGBA(r=0.1, g=0.5, b=1.0, a=0.9)
        se.lifetime = forever

        # ── delaunay edges ────────────────────────────────────────────────────
        de = Marker()
        de.header = Header(frame_id=self.frame_id, stamp=stamp)
        de.ns, de.id = "swagger_delaunay_edges", 2
        de.type, de.action = Marker.LINE_LIST, Marker.ADD
        de.scale.x = 0.03
        de.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4)
        de.lifetime = forever

        for u, v, data in self._G.edges(data=True):
            p1 = Point(x=self._G.nodes[u]["x"], y=self._G.nodes[u]["y"], z=0.08)
            p2 = Point(x=self._G.nodes[v]["x"], y=self._G.nodes[v]["y"], z=0.08)
            if data.get("edge_type", "skeleton") == "skeleton":
                se.points += [p1, p2]
            else:
                de.points += [p1, p2]

        ma = MarkerArray()
        ma.markers = [nm, se, de]
        self._graph_pub.publish(ma)
        self.get_logger().info("SWAGGER graph markers published on ~/graph")

    # ── helpers ─────────────────────────────────────────────────────────────

    def _nearest_node(self, x: float, y: float) -> int:
        dists = np.hypot(
            self._node_xy[:, 0] - x,
            self._node_xy[:, 1] - y,
        )
        return int(self._node_ids[int(np.argmin(dists))])

    def _dijkstra(
        self, sx: float, sy: float, gx: float, gy: float
    ) -> list[tuple[float, float]]:
        """
        1. Snap start/goal to nearest graph nodes.
        2. Run Dijkstra with composite edge cost.
        3. Return world-coord waypoints (start → graph path → goal).
        """
        src = self._nearest_node(sx, sy)
        dst = self._nearest_node(gx, gy)
        self.get_logger().debug(f"Dijkstra  src={src}  dst={dst}")

        try:
            node_path = nx.dijkstra_path(
                self._G,
                src,
                dst,
                weight=lambda u, v, d: _edge_cost(
                    self._G, u, v,
                    self.w_dist, self.w_sem, self.w_time,
                    self.speed, self.skel_only, self.delaunay_mul,
                ),
            )
        except nx.NetworkXNoPath:
            self.get_logger().warn(f"No path: node {src} → node {dst}")
            return []

        waypoints = [
            (self._G.nodes[n]["x"], self._G.nodes[n]["y"]) for n in node_path
        ]
        # prepend actual start pose, append actual goal pose
        return [(sx, sy)] + waypoints + [(gx, gy)]

    @staticmethod
    def _densify(
        wps: list[tuple[float, float]], resolution: float
    ) -> list[tuple[float, float]]:
        """
        Interpolate `wps` at `resolution`-metre spacing.
        Consecutive duplicate points are skipped to avoid division by zero.
        """
        if len(wps) < 2:
            return list(wps)

        dense: list[tuple[float, float]] = [wps[0]]
        for i in range(1, len(wps)):
            x0, y0 = wps[i - 1]
            x1, y1 = wps[i]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-9:
                continue
            steps = max(1, math.ceil(seg / resolution))
            for k in range(1, steps + 1):
                t = k / steps
                dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        return dense

    @staticmethod
    def _yaw_quat(x0: float, y0: float, x1: float, y1: float) -> Quaternion:
        yaw = math.atan2(y1 - y0, x1 - x0)
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def _to_path_msg(self, dense: list[tuple[float, float]]) -> Path:
        msg = Path()
        stamp = self.get_clock().now().to_msg()
        msg.header = Header(frame_id=self.frame_id, stamp=stamp)

        n = len(dense)
        for i, (x, y) in enumerate(dense):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position = Point(x=x, y=y, z=0.0)
            if i < n - 1:
                ps.pose.orientation = self._yaw_quat(
                    x, y, dense[i + 1][0], dense[i + 1][1]
                )
            else:
                # last pose inherits heading from previous
                ps.pose.orientation = msg.poses[-1].pose.orientation
            msg.poses.append(ps)
        return msg

    # ── Rviz goal_pose callback ──────────────────────────────────────────────

    def _cb_goal_pose(self, goal: PoseStamped) -> None:
        """
        Triggered by the Rviz '2D Goal Pose' tool (publishes to /goal_pose).
        Uses the latest AMCL pose as start (falls back to TF if AMCL not yet
        received), runs Dijkstra, and publishes the dense path on ~/path.
        """
        # ── start position: prefer AMCL pose (already in map frame) ─────────
        if self._amcl_pose is not None:
            sx = self._amcl_pose.pose.pose.position.x
            sy = self._amcl_pose.pose.pose.position.y
        else:
            # AMCL not yet received — fall back to TF
            try:
                tf = self._tf_buf.lookup_transform(
                    self.frame_id,
                    self._base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
                sx = tf.transform.translation.x
                sy = tf.transform.translation.y
            except TransformException as exc:
                self.get_logger().warn(
                    f"No robot pose available (AMCL not received, TF: {exc})"
                )
                return

        gx = goal.pose.position.x
        gy = goal.pose.position.y

        self.get_logger().info(
            f"Goal pose  ({gx:.2f}, {gy:.2f})  robot at ({sx:.2f}, {sy:.2f})"
        )

        waypoints = self._dijkstra(sx, sy, gx, gy)
        if not waypoints:
            return

        dense = self._densify(waypoints, self.res)
        self.get_logger().info(
            f"Path: {len(waypoints)} waypoints → {len(dense)} poses"
        )
        self._path_pub.publish(self._to_path_msg(dense))

    # ── service callback ─────────────────────────────────────────────────────

    def _cb_get_plan(
        self,
        request: GetPlan.Request,
        response: GetPlan.Response,
    ) -> GetPlan.Response:
        sx = request.start.pose.position.x
        sy = request.start.pose.position.y
        gx = request.goal.pose.position.x
        gy = request.goal.pose.position.y

        self.get_logger().info(
            f"Plan request  ({sx:.2f}, {sy:.2f}) → ({gx:.2f}, {gy:.2f})"
        )

        waypoints = self._dijkstra(sx, sy, gx, gy)
        if not waypoints:
            self.get_logger().warn("No path found for service request")
            response.plan = Path()
            return response

        dense = self._densify(waypoints, self.res)
        path = self._to_path_msg(dense)

        self.get_logger().info(
            f"Path: {len(waypoints)} graph waypoints → "
            f"{len(dense)} poses  (res={self.res * 100:.0f} cm)"
        )

        self._path_pub.publish(path)
        response.plan = path
        return response


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
