#include "quadruped_wheel/qw_state_machine.hpp"

#ifdef USE_SIMULATION
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    namespace backward{
        backward::SignalHandling sh;
    }
#endif

using namespace types;
MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(int argc, char* argv[]){
    std::cout << "State Machine Start Running" << std::endl;
    // Pass command line args to ROS2 so namespace (--ros-args -r __ns:=/M20_A) works
    rclcpp::init(argc, argv);

    // Select user-command source:
    //   --ros2-cmd → subscribe /M20/cmd_vel (autonomous navigation via path_follower)
    //   (default)  → keyboard  w/s/a/d
    RemoteCommandType rct = RemoteCommandType::kKeyBoard;
    for(int i = 1; i < argc; ++i){
        if(std::string(argv[i]) == "--ros2-cmd"){
            rct = RemoteCommandType::kROS2;
            std::cout << "Command mode: ROS2 /M20/cmd_vel" << std::endl;
            break;
        }
    }

    std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(RobotName::M20, rct);
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
