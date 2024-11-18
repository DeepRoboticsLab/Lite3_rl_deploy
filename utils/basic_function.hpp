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

inline float Deg2Rad(float deg){
    return deg /180.f * M_PI;
}

inline float Rad2Deg(float rad){
    return rad / M_PI * 180.;
}

inline Mat3f CrossVector(const Vec3f& r){
    Mat3f cm;
    cm << 0.f, -r(2), r(1),
        r(2), 0.f, -r(0),
        -r(1), r(0), 0.f;
    return cm;
}

inline float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
    if(t >= T) return xf;
    float a, b, c, d;
    d = x0;
    c = v0;
    a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
    b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
    return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
}

inline float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
    if(t >= T) return 0;
    float a, b, c;
    c = v0;
    a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
    b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
    return 3.*a*pow(t, 2) + 2.*b*t + c;
}
   
inline float LimitNumber(float data, float limit){
    limit = fabs(limit);
    if(data > limit) data = limit;
    else if(data < limit) data = -limit;
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