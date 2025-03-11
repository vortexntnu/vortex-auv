#ifndef JOY_PROCESSOR_AUV_HPP
#define JOY_PROCESSOR_AUV_HPP

#include "unordered_map"
#include "string"

Struct Gains
{
    double x: 0.0;
    double y: 0.0;
    double z: 0.0;
    double roll: 0.0;
    double pitch: 0.0;
    double yaw: 0.0;
}

class JoyProcessor:
{
    public:
        JoyProcessor(std::unordered_map<std::string, int> button_map, 
                     std::unordered_map<std::string, int> axis_map, 
                     Gains joy_gains, 
                     Gains reference_gains);
        ~JoyProcessor();

    private
}