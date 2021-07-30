// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

//basic declaration

int main(int argc, char* argv[]) {

  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim"); // :) lisence check
  #if WIN32
      timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
  #endif

  /// create raisim world
  double dt = 0.003; //set time interval.

  raisim::World world; 
  world.setTimeStep(dt);
  world.setERP(world.getTimeStep(), world.getTimeStep());
  world.addGround(0, "steel"); // set material of ground

  /// create objects
  
  auto arm1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\6dof2\\6dof2.urdf"); //:) raisim\\win32\\mt_debug\\bin 다음경로부터임
  auto arm2 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\iiwa\\iiwa14.urdf");

  raisim::CoordinateFrame raiframe;
  auto footFrameIndex = raiframe.getFrameIdxByName("joint1");

  raisim::Vec<3> footPosition, footVelocity, footAngularVelocity;
  raisim::Mat<3, 3> footOrientation;

  raiframe.getFramePosition(footFrameIndex, footPosition);
  raiframe.getFrameOrientation(footFrameIndex, footOrientation);
  raiframe.getFrameVelocity(footFrameIndex, footVelocity);
  raiframe.getFrameAngularVelocity(footFrameIndex, footAngularVelocity);

  Eigen::VectorXd jointNominalConfig(5), jointVelocityTarget(arm1->getDOF());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0; //set list of joint
  jointVelocityTarget.setZero(); //get arm1 Dof and set 0 ex) [0, 0, 0, 0, 0]

  Eigen::VectorXd jointPgain(arm1->getDOF()), jointDgain(arm1->getDOF());
  jointPgain.tail(5).setConstant(100.0); //set P gain -> tail(target link number).setConstant(P gain)
  jointDgain.tail(5).setConstant(5.0); //set D gain -> tail(target link number).setConstant(D gain)

  arm1->setGeneralizedCoordinate(jointNominalConfig);
  arm1->setGeneralizedForce(Eigen::VectorXd::Zero(arm1->getDOF()));
  arm1->setPdGains(jointPgain, jointDgain); //set PD gains
  arm1->setPdTarget(jointNominalConfig, jointVelocityTarget); //speed set zero to PD gain

  arm1->setName("arm_1"); //set name of object. it show in raisim
  arm2->setName("arm_2");
  

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  world.setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001); //:) material setting
  world.exportToXml(binaryPath.getDirectory(), "exportedWorld.xml");

  for (int i = 0; i < 20000; i++) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      
      //arm1->setGeneralizedVelocity(jointVel1);
      
      server.integrateWorldThreadSafe(); //:) 서버 유지
  }

  server.killServer();
}