/**
 * @file basic_function.hpp
 * @brief basic function
 * @author mazunwang
 * @version 1.0
 * @date 2024-06-05
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef BASIC_FUNCTION_HPP_
#define BASIC_FUNCTION_HPP_
#include "common_types.h"
#include "custom_types.h"

using namespace types;

namespace functions{
inline float LimitNumber(float data, float limit){
    limit = fabs(limit);
    if(data > limit) data = limit;
    else if(data < -limit) data = -limit;
    return data;
}

inline float LimitNumber(float data, float low, float high){
    if(low > high){
        std::cerr << "error limit range" << std::endl;
        return data;
    }
    if(data > high) return high;
    if(data < low) return low;
    return data;
}

inline Mat3f RpyToRm(const Vec3f &rpy){
    Mat3f rm;
    Eigen::AngleAxisf yawAngle(rpy(2), Vec3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(rpy(1), Vec3f::UnitY());
    Eigen::AngleAxisf rollAngle(rpy(0), Vec3f::UnitX());
    Eigen::Quaternion<float> q = yawAngle*pitchAngle*rollAngle;
    return q.matrix();
}

inline Vec3f RmToProjectedGravity(const Mat3f& rm) {        
    Vec3f gravity_world(0.f, 0.f, -1.f);   
    Vec3f gravity_base = rm.transpose() * gravity_world;
    return gravity_base;
}

inline float NormalizeAngle(float angle) {
    float result = std::fmod(angle, 2 * M_PI);
    
    if (result < 0) {
        result += 2 * M_PI;
    }
    
    if (result > M_PI) {
        result -= 2 * M_PI;
    }
    
    return result;
}

inline float Sign(const float &i){
    if (i > 0) {
        return 1.;
    } else if (i == 0) {
        return 0.;
    } else {
        return -1.;
    }
}
};//namespace

#endif