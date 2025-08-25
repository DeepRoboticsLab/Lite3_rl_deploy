
#include "state_machine.hpp"
#ifdef BUILD_SIMULATION
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    namespace backward{
        backward::SignalHandling sh;
    }
#endif
using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(){
    StateMachine state_machine(RobotType::Lite3);
    state_machine.Run();
    return 0;
}