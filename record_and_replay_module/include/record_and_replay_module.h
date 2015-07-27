/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef record_and_replay_module_H_
#define record_and_replay_module_H_

#include "RobotLib/RobotInterface.h"
#include "KUKARobotModel/LWRRobot.h"

#include "RobotLib/Sensor.h"
#include "RobotLib/KinematicChain.h"

#include <vector>
#include "MotionGenerators/CDDynamics.h"


class record_and_replay_module : public RobotInterface
{
    // a vector to store the recorded joint space trajectory
    std::vector<MathLib::Vector> trajectory_;
    // and integer to keep track of where we are at in the recorded trajectory
    size_t traj_ind_;
    // just an enum to keep track of what the controller is doing in its main loop
    enum CurrentState {NONE,RECORD,PREPARE,REPLAY};
    CurrentState current_state_;
    // a pointer to the lwr_robot which carries specific kuka functionalities like cartesian or joint space impedance control
    LWRRobot * lwr_robot_;
    // an object that will get the values of the joint encoders
    RevoluteJointSensorGroup joint_sensors_;
    // an object to which we will be able to give the commands to the robot
    RevoluteJointActuatorGroup joint_actuators_;
    // an object that we will use to generate smooth interpolated trajectories to the starting point of the recorded traj
    CDDynamics * cd_dyn_;
    // Distance threshold for reaching the target.
    double target_tolerance_;

    // just a little bool to keep track of when we start and stop a new state of the robot
    bool bFirst_;

    // these are two vectors we will use to store stiffness values for gravity compensating and moving respectively
    MathLib::Vector gravcomp_stiffness_;
    MathLib::Vector moving_stiffness_;




public:
            record_and_replay_module();
    virtual ~record_and_replay_module();

    virtual Status              RobotInit();
    virtual Status              RobotFree();

    virtual Status              RobotStart();
    virtual Status              RobotStop();

    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

            virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
private:
            void SwitchState(CurrentState next_state);
};



#endif
