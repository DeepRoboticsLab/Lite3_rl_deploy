
#include "state_machine.hpp"

using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(){
    StateMachine state_machine(RobotType::Lite3);
    state_machine.Run();
    return 0;
}