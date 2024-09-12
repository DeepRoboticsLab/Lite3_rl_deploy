#ifndef JUEYING_RAISIM_SIMULATION_HPP_
#define JUEYING_RAISIM_SIMULATION_HPP_

#include "simulation/raisim_interface.hpp"

namespace interface{
class JueyingRaisimSimulation : public RaisimInterface{
public:
    JueyingRaisimSimulation(const std::string& ak, const std::string& up, const std::string& rn="x30_sim"):RaisimInterface(rn, 12){
        this->SetActivationKey(ak);
        this->SetUrdfModelPath(up);
    }
    ~JueyingRaisimSimulation(){

    }
};
};
#endif