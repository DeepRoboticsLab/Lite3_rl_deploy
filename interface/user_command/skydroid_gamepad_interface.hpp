#ifndef SKYDROID_GAMEPAD_INTERFACE_HPP_
#define SKYDROID_GAMEPAD_INTERFACE_HPP_

#include "user_command_interface.h"
#include "skydroid_gamepad.h"
#include "custom_types.h"

using namespace interface;
using namespace types;

class SkydroidGamepadInterface : public UserCommandInterface{
private:
    std::shared_ptr<SkydroidGamepad> gamepad_ptr_;
    SkydroidKeys sd_keys_record_, sd_keys_;
    UserCommand usr_cmd_;
    bool IsKeysEqual(const SkydroidKeys& a, const SkydroidKeys& b){
        if(a.keys_value != b.keys_value) return false;
        for(int i=0;i<kAxisChannlSize;++i){
            if(a.axis_values[i] != b.axis_values[i]) return false;
        }
        for(int i=0;i<kSwitchKeysSize;++i){
            if(a.switch_keys[i] != b.switch_keys[i]) return false;
        }
        return true;
    }


    bool transform_cmd_flag_ = true;
    std::thread transform_thread_;
    bool first_flag_ = true;
    
    
public:
    SkydroidGamepadInterface(int port);
    ~SkydroidGamepadInterface();

    virtual void Start();
    virtual void Stop();
    virtual UserCommand GetUserCommand(); 

    void TransformSkydroidToUserCommand();
    
    void SetMotionStateFeedback(const MotionStateFeedback& msfb){
        msfb_ = msfb;
    }
    void PrintGamepadData(SkydroidKeys *data){
        std::cout << "\nAxis value: \t";
        for(auto i : data->axis_values) std::cout << i << ",\t";
        std::cout << "\nSW keys: \t";
        std::cout << bool(data->sw1) << ",\t" << bool(data->sw2) << ",\t" << bool(data->sw3) << ",\t" << bool(data->sw4) << ",\t";
        std::cout << "\nDownside keys: \t";
        std::cout << bool(data->A) << ",\t" << bool(data->B) << ",\t" << bool(data->C) << ",\t" << bool(data->D) << ",\t";
        std::cout << bool(data->E) << ",\t" << bool(data->F) << ",\t" << bool(data->right) << ",\t";
        std::cout << "\n";  
    }


};

SkydroidGamepadInterface::SkydroidGamepadInterface(int port):UserCommandInterface(){
    std::cout << "Using Skydroid Gamepad Command Interface" << std::endl;
    gamepad_ptr_ = std::make_shared<SkydroidGamepad>(port);
    std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
}

SkydroidGamepadInterface::~SkydroidGamepadInterface(){
}

void SkydroidGamepadInterface::Start(){
    gamepad_ptr_->StartDataThread();
    transform_thread_ = std::thread(std::bind(&SkydroidGamepadInterface::TransformSkydroidToUserCommand, this));
}

void SkydroidGamepadInterface::Stop(){
    gamepad_ptr_->StopDataThread();
    transform_cmd_flag_ = false;
    transform_thread_.join();
}

void SkydroidGamepadInterface::TransformSkydroidToUserCommand(){
    while (transform_cmd_flag_) {
        sd_keys_ = gamepad_ptr_->GetKeys();
        // PrintGamepadData(&sd_keys_);
        if(first_flag_) {
            sd_keys_record_ = sd_keys_;
            first_flag_ = false;
            continue;
        }
        // std::cout << "axis: " << int(sd_keys_.A) << std::endl;
        usr_cmd_.forward_vel_scale = sd_keys_.left_axis_y;
        usr_cmd_.side_vel_scale = -sd_keys_.left_axis_x;
        usr_cmd_.turnning_vel_scale = -sd_keys_.right_axis_x;
        if (!IsKeysEqual(sd_keys_, sd_keys_record_)) {
            switch (msfb_.current_state){
            case RobotMotionState::WaitingForStand:
                if(sd_keys_.A != sd_keys_record_.A) {
                    usr_cmd_.target_mode = int(RobotMotionState::StandingUp); 
                }
                break;
            case RobotMotionState::StandingUp:
                if(sd_keys_.right != sd_keys_record_.right){
                    usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                }
                break;
            
            default:
                break;
            }
            if(sd_keys_.sw1==2&&sd_keys_.sw4==2){
                usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
            }
            sd_keys_record_ = sd_keys_;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
}


UserCommand SkydroidGamepadInterface::GetUserCommand(){
    return usr_cmd_;
}

// bool SkydroidGamepadInterface::IsKeysEqual(const SkydroidKeys& a, const SkydroidKeys& b)

// void SkydroidGamepadInterface::PrintGamepadData(SkydroidKeys *data)
#endif