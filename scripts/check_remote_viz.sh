#!/usr/bin/env bash
# =============================================================================
# check_remote_viz.sh — why can't the laptop see the robot's ROS topics?
#
#   bash scripts/check_remote_viz.sh                 # run ON THE LAPTOP
#   GOS=10.21.31.104 AOS=192.168.8.103 bash scripts/check_remote_viz.sh
#
# Walks the stack BOTTOM-UP and stops at the first layer that is actually
# broken, because every layer above it will look broken too and fixing the top
# one never helps:
#
#   0  env        RMW / ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY / CYCLONEDDS_URI
#   1  route      is there any IP path to the GOS at all
#   2  reach      does the GOS answer, and by the route we think
#   3  interface  does CYCLONEDDS_URI pin an address that exists HERE
#   4  discovery  do SPDP datagrams actually arrive at the GOS
#   5  topics     ros2 topic list / hz
#   6  bandwidth  is anything subscribed that cannot fit the wifi hop
#
# The classic failure is layer 1 or 4 presenting as layer 5: `ros2 topic list`
# prints nothing, which looks like a DDS problem and is almost always routing or
# a firewall. Layers 4-6 print the paired command to run on the OTHER host,
# since neither side can prove a one-way UDP path alone.
#
# See config/cyclonedds_laptop.xml for the two supported topologies (routed via
# the AOS, or cabled into the robot's internal switch) and the one-time AOS
# iptables/sysctl setup the routed one needs.
# =============================================================================
set -uo pipefail

GOS="${GOS:-10.21.31.104}"        # nav_core, on the robot's internal switch
AOS="${AOS:-192.168.8.103}"       # the robot host that routes between the two
DOMAIN="${ROS_DOMAIN_ID:-0}"
SPDP_PORT=$(( 7400 + 250 * DOMAIN + 10 ))    # index 0; +2 per participant index

ok()   { printf '  \033[32mOK\033[0m    %s\n' "$*"; }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; }
warn() { printf '  \033[33mWARN\033[0m  %s\n' "$*"; }
note() { printf '        %s\n' "$*"; }
hdr()  { printf '\n== %s ==\n' "$*"; }

die() { printf '\n\033[31mStop here.\033[0m Fix the FAIL above; layers above it cannot work.\n'; exit 1; }

# ---------------------------------------------------------------- 0  env ----
hdr "0. environment"
[[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" ]] \
  && ok "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
  || { bad "RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-<unset>}' — nav_core is Cyclone; a mismatch discovers NOTHING"
       note "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"; }

ok "ROS_DOMAIN_ID=$DOMAIN (GOS containers use 0; SPDP base port $SPDP_PORT)"

if [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]]; then
  bad "ROS_LOCALHOST_ONLY=1 — pins the RMW to loopback, nothing remote is visible"
  note "unset ROS_LOCALHOST_ONLY"
else
  ok "ROS_LOCALHOST_ONLY unset/0"
fi

URI="${CYCLONEDDS_URI:-}"
if [[ -z "$URI" ]]; then
  warn "CYCLONEDDS_URI unset — Cyclone will pick an interface ARBITRARILY between"
  note "equal-quality candidates (wifi vs docker0 vs tailscale0). Point it at"
  note "config/cyclonedds_laptop.xml so the choice is pinned."
else
  ok "CYCLONEDDS_URI=$URI"
fi

# -------------------------------------------------------------- 1  route ----
hdr "1. route to the GOS ($GOS)"
for t in ip ping; do
  command -v "$t" >/dev/null 2>&1 || {
    bad "'$t' not found — cannot test the network layer, and a missing tool must"
    note "not be read as a passing check. Install iproute2/iputils-ping, or run"
    note "this on the laptop HOST rather than inside a slim container:"
    note "  sudo apt install -y iproute2 iputils-ping"
    die; }
done

RT=$(ip route get "$GOS" 2>/dev/null) || RT=""
if [[ -z "$RT" ]]; then
  bad "no route at all to $GOS"
  note "  [LAPTOP]  sudo ip route add 10.21.31.0/24 via $AOS"
  note "Run that ONLY here. The AOS must NOT get this route: it already sits on"
  note "10.21.31.0/24, and $AOS is its own address, so the route points the subnet"
  note "back at itself and the AOS stops reaching the GOS entirely."
  die
fi
note "$RT"
VIA=$(awk '{for(i=1;i<=NF;i++) if($i=="via") print $(i+1)}' <<<"$RT")
DEV=$(awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}' <<<"$RT")
if [[ "$VIA" == "$AOS" ]]; then
  ok "routed via the AOS ($AOS) — the intended multihop path"
elif [[ -z "$VIA" && -n "$DEV" ]]; then
  ok "on-link via $DEV (same L2 as the GOS — the cabled-into-the-switch topology)"
elif [[ -z "$VIA" ]]; then
  bad "could not parse a route from: $RT"; die
else
  bad "routed via $VIA, which is NOT the AOS and almost certainly has no route"
  note "to 10.21.31.0/24 (this is the default-gateway trap). Add the specific route:"
  note "  [LAPTOP]  sudo ip route add 10.21.31.0/24 via $AOS"
  note "Run that ONLY here — never on the AOS, which already sits on that subnet."
  die
fi

# -------------------------------------------------------------- 2  reach ----
hdr "2. reachability"
if ping -c2 -W2 "$AOS" >/dev/null 2>&1; then ok "AOS $AOS answers"
else bad "AOS $AOS unreachable — check wifi association first"; die; fi

if ping -c2 -W2 "$GOS" >/dev/null 2>&1; then
  ok "GOS $GOS answers"
else
  bad "GOS $GOS unreachable although a route exists."
  note "A ping needs the request forwarded OUT and the reply routed BACK, and from"
  note "here those failures are indistinguishable. Do NOT start applying fixes."
  note "Triage in THIS ORDER; each step rules out everything after it."
  note ""
  note " 1. Can the ROUTER itself reach the GOS?  (does not involve this laptop)"
  note "      [AOS]  ping -c3 $GOS"
  note "      [AOS]  ip route get $GOS       # which device does it pick?"
  note "      [AOS]  ip neigh show $GOS      # REACHABLE, or FAILED?"
  note "    Fails -> stop. The GOS is down, has changed address, or its NIC is"
  note "    unplugged. A packet with no resolvable next hop is dropped BEFORE it is"
  note "    transmitted, so it never appears on the outbound interface — which is"
  note "    why this looks like a forwarding fault but is not one. And if route get"
  note "    picks a device other than the robot's internal NIC, every '-o <nic>'"
  note "    firewall rule is dead code and the chain policy applies instead."
  note ""
  note " 2. Do the requests even ARRIVE?  Watch ALL interfaces, not just one:"
  note "      [AOS]  sudo tcpdump -ni any icmp and host ${SRC:-<laptop-ip>}"
  note "    nothing at all        -> laptop-side or wifi/AP problem, not the AOS."
  note "    arrives, never leaves -> forwarding/firewall; go to step 3."
  note "    leaves, no reply      -> GOS has no route back; go to step 4."
  note ""
  note " 3. Is a DROP actually counting?  Counters beat reading rules:"
  note "      [AOS]  sudo iptables -vnL FORWARD --line-numbers"
  note "    Watch the 'policy DROP' counter while the ping runs. iptables is"
  note "    FIRST-MATCH-WINS: a broad ACCEPT above a narrow RELATED,ESTABLISHED"
  note "    rule makes the narrow one dead code, so read the chain IN ORDER"
  note "    rather than grepping for the rule you expect to matter."
  note "      [AOS]  sysctl net.ipv4.ip_forward                 # want 1"
  note "      [AOS]  sysctl net.ipv4.conf.p2p0.forwarding       # per-if override"
  note ""
  note " 4. Does the GOS know the way back?  'ip route' edits do NOT survive a"
  note "    reboot, which makes this the most common regression here:"
  note "      [GOS]  ip route get ${SRC:-<laptop-ip>}"
  note "      [GOS]  sudo ip route replace 192.168.8.0/24 via $AOS"
  note "      [GOS]  sudo ufw status"
  note "    Persist it in /etc/netplan/*.yaml once it works."
  note ""
  note "If ICMP is filtered but UDP is not, ping can fail while DDS works. Confirm"
  note "by watching for SPDP instead of ICMP — see layer 4 below."
  die
fi

# ---------------------------------------------------------- 3  interface ----
hdr "3. does the config pin an address that exists here"
SRC=$(awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' <<<"$RT")
[[ -n "$SRC" ]] && ok "kernel will source GOS traffic from $SRC"

if [[ -n "$URI" ]]; then
  F="${URI#file://}"
  if [[ -r "$F" ]]; then
    PINNED=$(grep -oE 'NetworkInterface address="[^"]+"' "$F" | grep -oE '[0-9.]+' | tr '\n' ' ')
    if [[ -n "$PINNED" ]]; then
      for a in $PINNED; do
        if ip -br addr | grep -q "$a"; then
          ok "pinned $a exists on this machine"
          [[ -n "$SRC" && "$a" != "$SRC" ]] && warn "but the GOS route uses src $SRC — Cyclone will bind the wrong NIC"
        else
          bad "pinned $a does NOT exist here. Cyclone REFUSES TO START on this:"
          note "  '$a: does not match an available interface'"
          note "  'dds_create_participant(domain -1) failed: -1'"
          note "Edit <Interfaces> in $F to $SRC (or cable in and use 10.21.31.50)."
          die
        fi
      done
    fi
    if grep -qE '<Peer address="[0-9.]+:[0-9]+"' "$F"; then
      warn "a <Peer> names an explicit PORT. That probes ONE participant, and SPDP"
      note "has no relay — nav_core + m20_bridge are ~20 separate processes, each"
      note "its own participant on its own port. Expect a partial topic list."
      note "Drop the ':port' suffix so all indices 0..MaxAutoParticipantIndex are probed."
    fi
    grep -qE '<MaxAutoParticipantIndex>' "$F" \
      || warn "no <MaxAutoParticipantIndex>: Cyclone defaults to 9, but the robot's own Fast-DDS holds indices 0-9 and the GOS containers land at 10+. Set 60."
  else
    warn "CYCLONEDDS_URI points at $F which is not readable here"
  fi
fi

# ---------------------------------------------------------- 4  discovery ----
hdr "4. do discovery datagrams reach the GOS"
note "UDP gives no handshake to test, so confirm it on the GOS itself."
note "RUN ON THE GOS (10.21.31.104) while this laptop has rviz2/ros2 running:"
note ""
note "  sudo tcpdump -ni any udp portrange $SPDP_PORT-$(( SPDP_PORT + 120 )) and host ${SRC:-<laptop-ip>}"
note ""
note "Expect inbound SPDP from ${SRC:-<laptop-ip>}. Then check the SOURCE address:"
note "  src ${SRC:-<laptop-ip>}   -> good, NAT is correctly exempted"
note "  src $AOS  -> the AOS is masquerading robot<->laptop traffic; the RTPS"
note "                       locators inside still say $GOS, which makes every"
note "                       discovery problem unreadable. On the AOS:"
note "    sudo iptables -t nat -I POSTROUTING 1 -s 10.21.31.0/24 -d 192.168.8.0/24 -j ACCEPT"
note ""
note "Nothing at all inbound -> layer 1/2 is lying; recheck FORWARD on the AOS."
note "Inbound but no reply    -> add this laptop to <Peers> in cyclonedds_gos.xml"
note "                           (multicast SPDP cannot cross the AOS hop)."

# ------------------------------------------------------------- 5  topics ----
hdr "5. topics"
if ! command -v ros2 >/dev/null 2>&1; then
  warn "no ros2 on PATH — run this inside the m20-deploy:humble container"
else
  note "restarting the daemon: it caches discovery and will keep serving a stale"
  note "empty list long after the network is fixed."
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  mapfile -t TOPICS < <(timeout 15 ros2 topic list 2>/dev/null | grep -v '^/parameter_events$\|^/rosout$')
  if (( ${#TOPICS[@]} == 0 )); then
    bad "no topics discovered"
    note "Re-run with Cyclone tracing to see the interface and peers it really used:"
    note "  sed -i 's|<Verbosity>warning|<Verbosity>config|' ${URI#file://}"
    note "  ros2 daemon stop; ros2 topic list; # then read the trace"
    die
  fi
  ok "${#TOPICS[@]} topics discovered"
  for t in /scan /odom /map /tf /odometry/filtered /global_costmap/costmap; do
    printf '%s\n' "${TOPICS[@]}" | grep -qx "$t" && ok "  $t" || warn "  $t MISSING"
  done
  note ""
  note "A PARTIAL list (some nodes, not others) is the pinned-peer-port symptom"
  note "from layer 3, not a flaky network."
fi

# ---------------------------------------------------------- 6  bandwidth ----
hdr "6. bandwidth over the wifi hop"
warn "custom_nav2.rviz ships a PointCloud2 display on /lidar/points."
note "~1.2 MB x 10 Hz = ~96 Mbit/s. That does not fit this hop and it starves"
note "the nav topics of airtime — RViz then looks 'slow' for a reason that has"
note "nothing to do with DDS config. UNTICK that display for remote viz; use"
note "/scan (a few kB) instead. Verify what you are actually pulling:"
note "  ros2 topic bw /lidar/points     # only if you deliberately want it"
note "  ros2 topic bw /global_costmap/costmap"
note ""
note "The global costmap is 514x663 cells and navigation_params.yaml sets"
note "always_send_full_costmap: true -> ~341 kB every second. Tolerable, but if"
note "costmap viz is what stutters, that is the topic to drop first."
note ""
note "Also note cyclonedds_gos.xml sets MaxMessageSize 65500B, tuned for the"
note "loopback cloud path. Across a 1500-MTU wifi link each such datagram IP-"
note "fragments ~45 ways and ONE lost fragment discards the whole 64 kB chunk."
note "It only bites on big remote samples, which is one more reason to leave the"
note "cloud display off rather than to retune it."

printf '\nDone. Layers 0-3 and 5 are checked here; 4 and 6 need the paired\ncommands above, run on the GOS.\n'
