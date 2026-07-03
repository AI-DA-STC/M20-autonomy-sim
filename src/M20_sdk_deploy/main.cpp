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
    rclcpp::init(argc, argv);

    // Pass --autonomy flag to use /cmd_vel from CMU autonomy stack
    // instead of keyboard.  e.g.:  ros2 run rl_deploy rl_deploy -- --autonomy
    bool use_autonomy = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--autonomy") { use_autonomy = true; break; }
    }

    RemoteCommandType cmd_type = use_autonomy
        ? RemoteCommandType::kROS2CmdVel
        : RemoteCommandType::kKeyBoard;

    if (use_autonomy)
        std::cout << "[INFO] Autonomy mode: reading /cmd_vel from CMU stack\n";
    else
        std::cout << "[INFO] Keyboard mode: use W/S/A/D/Q/E\n";

    std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(
        RobotName::M20, cmd_type);
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
