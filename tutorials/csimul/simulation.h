#ifndef __SIMULATION_H__
#define __SIMULATION_H__
#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

#include "trial/fsm.h"

class Simulation
{
public:
    Simulation();
    void init();
    void reset();
    void step(bool);

    std::shared_ptr<World> world;
    //std::vector<std::shared_ptr<VPC::Record>> records;
    //std::string output_path;

    int getStateNum();
    int getActionNum();
    boost::python::list getState();
    void setAction(boost::python::list action);
    double getReward();
    int getDone();

    Eigen::VectorXd mDesiredDofs[4];
    std::shared_ptr<fsm> fsm;
    std::shared_ptr<> controller;
    double reward;
};

#endif
