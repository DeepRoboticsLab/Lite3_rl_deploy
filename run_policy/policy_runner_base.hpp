/**
 * @file policy_runner_base.hpp
 * @brief policy runner base class
 * @author mazunwang
 * @version 1.0
 * @date 2024-06-06
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef POLICY_RUNNER_BASE_HPP_
#define POLICY_RUNNER_BASE_HPP_


#include "common_types.h"

using namespace types;

class PolicyRunnerBase{
public:
    PolicyRunnerBase(std::string policy_name):policy_name_(policy_name){
    }
    virtual ~PolicyRunnerBase(){}
    /**
     * @brief use to display policy detail
     */
    virtual void DisplayPolicyInfo() = 0;

    /**
     * @brief Get the robot action object by run your policy
     * @return RobotAction 
     */
    virtual RobotAction GetRobotAction(const RobotBasicState&) = 0;

    /**
     * @brief execute function when first entering policy runner
     */
    virtual void OnEnter() = 0;

    /**
     * @brief Set the decimation
     * @param  d decimation
     */
    virtual void SetDecimation(int d){
        decimation_ = d;
    }

    const std::string policy_name_;
    int decimation_;
    int run_cnt_;
};






#endif