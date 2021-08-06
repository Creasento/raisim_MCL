// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

#include <string>
#include <iostream>
#include <fstream>

using namespace std;

//basic declaration
int main(int argc, char* argv[]) {

  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim"); // :) lisence check
  #if WIN32
      timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
  #endif

  /// create raisim world
  double dt = 0.001; //set time interval.

  raisim::World world; 
  world.setTimeStep(dt);
  world.setERP(world.getTimeStep(), world.getTimeStep());
  world.addGround(0, "steel"); // set material of ground

  /// create objects
  auto arm1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\6dof2\\6dof2.urdf"); //:) raisim\\win32\\mt_debug\\bin 다음경로부터임
  auto arm2 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\iiwa\\iiwa14.urdf");

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
  
  auto footFrameIndex1 = arm1->getFrameIdxByName("joint1");
  auto footFrameIndex2 = arm1->getFrameIdxByName("joint2");
  auto footFrameIndex3 = arm1->getFrameIdxByName("joint3");
  auto footFrameIndex4 = arm1->getFrameIdxByName("joint4");
  auto footFrameIndex5 = arm1->getFrameIdxByName("joint5");

  raisim::Vec<3> footPosition1, footVelocity1, footAngularVelocity1;
  raisim::Vec<3> footPosition2, footVelocity2, footAngularVelocity2;
  raisim::Vec<3> footPosition3, footVelocity3, footAngularVelocity3;
  raisim::Vec<3> footPosition4, footVelocity4, footAngularVelocity4;
  raisim::Vec<3> footPosition5, footVelocity5, footAngularVelocity5;

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  world.setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001); //:) material setting
  //world.exportToXml(binaryPath.getDirectory(), "exportedWorld.xml");

  /// save joint variables to txt
  ofstream fout("joint.txt", ios_base::out);

  for (int i = 0; i < 2000; i++) {

      std::this_thread::sleep_for(std::chrono::microseconds(1));
      //joint1 read
      arm1->getFramePosition(footFrameIndex1, footPosition1);
      arm1->getFrameVelocity(footFrameIndex1, footVelocity1);
      arm1->getFrameAngularVelocity(footFrameIndex1, footAngularVelocity1);
      //joint2 read
      arm1->getFramePosition(footFrameIndex2, footPosition2);
      arm1->getFrameVelocity(footFrameIndex2, footVelocity2);
      arm1->getFrameAngularVelocity(footFrameIndex2, footAngularVelocity2);
      //joint3 read
      arm1->getFramePosition(footFrameIndex3, footPosition3);
      arm1->getFrameVelocity(footFrameIndex3, footVelocity3);
      arm1->getFrameAngularVelocity(footFrameIndex3, footAngularVelocity3);
      //joint4 read
      arm1->getFramePosition(footFrameIndex4, footPosition4);
      arm1->getFrameVelocity(footFrameIndex4, footVelocity4);
      arm1->getFrameAngularVelocity(footFrameIndex4, footAngularVelocity4);
      //joint5 read
      arm1->getFramePosition(footFrameIndex5, footPosition5);
      arm1->getFrameVelocity(footFrameIndex5, footVelocity5);
      arm1->getFrameAngularVelocity(footFrameIndex5, footAngularVelocity5);

      //joint1 txt
      fout << "Position of Joint1, xyz, time(ms) = ";
      fout << i << endl;
      fout << footPosition1.e() << endl; //cout the position(xyz) of joint3

      fout << "Velocity of Joint1, xyz, time(ms) = ";
      fout << i << endl;
      fout << footVelocity1.e() << endl;

      fout << "Angular velocity of Joint1, rpy, time(ms) = ";
      fout << i << endl;
      fout << footAngularVelocity1.e() << endl;
      //joint2 txt
      fout << "Position of Joint2, xyz, time(ms) = ";
      fout << i << endl;
      fout << footPosition2.e() << endl; //cout the position(xyz) of joint3

      fout << "Velocity of Joint2, xyz, time(ms) = ";
      fout << i << endl;
      fout << footVelocity2.e() << endl;

      fout << "Angular velocity of Joint2, rpy, time(ms) = ";
      fout << i << endl;
      fout << footAngularVelocity2.e() << endl;
      //joint3 txt
      fout << "Position of Joint3, xyz, time(ms) = ";
      fout << i << endl;
      fout << footPosition3.e() << endl; //cout the position(xyz) of joint3

      fout << "Velocity of Joint3, xyz, time(ms) = ";
      fout << i << endl;
      fout << footVelocity3.e() << endl;

      fout << "Angular velocity of Joint3, rpy, time(ms) = ";
      fout << i << endl;
      fout << footAngularVelocity3.e() << endl;
      //joint4 txt
      fout << "Position of Joint4, xyz, time(ms) = ";
      fout << i << endl;
      fout << footPosition4.e() << endl; //cout the position(xyz) of joint3

      fout << "Velocity of Joint4, xyz, time(ms) = ";
      fout << i << endl;
      fout << footVelocity4.e() << endl;

      fout << "Angular velocity of Joint4, rpy, time(ms) = ";
      fout << i << endl;
      fout << footAngularVelocity4.e() << endl;
      //joint5 txt
      fout << "Position of Joint5, xyz, time(ms) = ";
      fout << i << endl;
      fout << footPosition5.e() << endl; //cout the position(xyz) of joint3

      fout << "Velocity of Joint5, xyz, time(ms) = ";
      fout << i << endl;
      fout << footVelocity5.e() << endl;

      fout << "Angular velocity of Joint5, rpy, time(ms) = ";
      fout << i << endl;
      fout << footAngularVelocity5.e() << endl;

      //arm1->setGeneralizedVelocity(jointVel1);
      
      server.integrateWorldThreadSafe(); //:) 서버 유지
  }
  fout.close();
  server.killServer();
}