#include "record_and_replay_module.h"

using namespace std;

record_and_replay_module::record_and_replay_module()
    :RobotInterface(){
}
record_and_replay_module::~record_and_replay_module(){
}

RobotInterface::Status record_and_replay_module::RobotInit(){

  print("Record & Replay module::init");

  // initializeing the robot. This is a hack to avoid seg faults in case we run on simulation
  if(mRobot->IsSimulationMode()){
    lwr_robot_ = new LWRRobot;
  }else{
    lwr_robot_ = (LWRRobot*) mRobot;
    lwr_robot_->SetControlMode(Robot::CTRLMODE_JOINTIMPEDANCE);
  }
  // set up the joint sensors
  joint_sensors_.SetSensorsList(lwr_robot_->GetSensors());
  // set up the joint actuators
  joint_actuators_.SetActuatorsList(lwr_robot_->GetActuators());


  gravcomp_stiffness_.Resize(7);
  gravcomp_stiffness_.Zero();

  moving_stiffness_.Resize(7);
  // lets use the default values from the lwr_robot
  moving_stiffness_ = lwr_robot_->GetJointStiffness();

  traj_ind_ = 0;

  const double dt = 0.002;
  const double wn = 3;
  cd_dyn_ = new CDDynamics(7, dt, wn);
  target_tolerance_ = 0.005;

  MathLib::Vector velLimits(7);
  velLimits.One();
  velLimits *= 1.5;
  cd_dyn_->SetVelocityLimits(velLimits);


  // finally define some commands that allows to interract with the controller via the console
  AddConsoleCommand("gravcomp");
  AddConsoleCommand("record");
  AddConsoleCommand("stop");
  AddConsoleCommand("replay");
  return STATUS_OK;
}
RobotInterface::Status record_and_replay_module::RobotFree(){
  return STATUS_OK;
}
RobotInterface::Status record_and_replay_module::RobotStart(){
  return STATUS_OK;
}
RobotInterface::Status record_and_replay_module::RobotStop(){
  return STATUS_OK;
}
RobotInterface::Status record_and_replay_module::RobotUpdate(){
  // for the kuka robot, this is redunant with RobotUpdateCore.
  return STATUS_OK;
}
RobotInterface::Status record_and_replay_module::RobotUpdateCore(){
  // this is the main loop. Every 1/500 second this will be called, and any new control command set to lwr_robot_ will then be applied to the robot
  // lets start by updating the position sensors
  joint_sensors_.ReadSensors();

  // now, what we should do here depends on which state we are in..
  switch(current_state_){
    {
      case NONE:
        if(bFirst_)
          bFirst_ = false;
        // here we should just gravity compensate. Therefore, we set zero stiffness
        lwr_robot_->SetJointStiffness(gravcomp_stiffness_);
        // also we had better feed back the actual joint positions as targets to avoid annoying low-level kuka errors..
        joint_actuators_.SetJointPositions(joint_sensors_.GetJointPositions());
        break;
    }
    {
      case RECORD:
        // here we should do exactly the same except now we also record the data coming from the sensors
        if(bFirst_){
          trajectory_.clear();
          lwr_robot_->SetJointStiffness(gravcomp_stiffness_);
          bFirst_ = false;
        }
        trajectory_.push_back(joint_sensors_.GetJointPositions());
        // also we had better feed back the actual joint positions as targets to avoid annoying low-level kuka errors..
        joint_actuators_.SetJointPositions(joint_sensors_.GetJointPositions());
        break;
    }
    {
      case PREPARE:
        // we prepare to replay by moving smoothly to the first configuration in the recorded trajectory
        if(bFirst_){
          // set the target configuration for the trajectory interpolator
          cd_dyn_->SetTarget(trajectory_[0]);
          // initialize the trajectory interpolator with our current position
          cd_dyn_->SetState(joint_sensors_.GetJointPositions());
          // but now we want to move! so we need to have some stiffness..
          lwr_robot_->SetJointStiffness(moving_stiffness_);
          bFirst_ = false;
        }
        // invoke the internal update of the trajectory interpolator
        cd_dyn_->Update();
        // extract the current position from the trajectory interpolator
        MathLib::Vector desired_position(7);
        cd_dyn_->GetState(desired_position);
        // .. and apply it to the robot as desired position command
        joint_actuators_.SetJointPositions(desired_position);
        // finally we check if we are close to the starting configuration, in
        // which case we can switch to replaying the trajectory
        if((joint_sensors_.GetJointPositions() - trajectory_[0]).Norm() < target_tolerance_)
          SwitchState(REPLAY);
        break;
    }
    {
      case REPLAY:
        if(bFirst_){
          traj_ind_ = 0;
          bFirst_ = false;
        }
        // apply the desired positions and increment
        joint_actuators_.SetJointPositions(trajectory_[traj_ind_]);
        traj_ind_++;
        // if we finished replaying, switch back to prepare
        if(traj_ind_>trajectory_.size()-1){
          SwitchState(PREPARE);
        }

        break;

    }
  }

  joint_actuators_.WriteActuators();
  return STATUS_OK;
}

void record_and_replay_module::SwitchState(CurrentState next_state){
  current_state_ = next_state;
  bFirst_ = true;
}

int record_and_replay_module::RespondToConsoleCommand(const string cmd, const vector<string> &args){
  // here we have to define what happens when the commands defined in the Init method are issued
  if(cmd=="gravcomp") {
    print("In gravcomp mode.");
    SwitchState(NONE);
  }
  else if(cmd=="record") {
    print("Recording trajectory.");
    SwitchState(RECORD);
  }
  else if (cmd=="stop") {
    print("In gravcomp mode.");
    SwitchState(NONE);
  }
  else if(cmd=="replay") {
    print("Replaying trajectory.");
    SwitchState(PREPARE);
  }


  return 0;
}

void record_and_replay_module::print(const char *str) {
  this->GetConsole()->Print(str);
}


extern "C"{
  // These two "C" functions manage the creation and destruction of the class
  record_and_replay_module* create(){return new record_and_replay_module();}
  void destroy(record_and_replay_module* module){delete module;}
}
