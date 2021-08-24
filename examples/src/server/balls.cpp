// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

#include <time.h>
#include <iostream>
#include <fstream>

using namespace std;

int runTime = 30000;
int runTime2 = runTime * 2;
int runTime3 = runTime * 3;

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
  
  Eigen::VectorXd positionJ1(runTime3);
  Eigen::VectorXd positionJ2(runTime3);
  Eigen::VectorXd positionJ3(runTime3);
  Eigen::VectorXd positionJ4(runTime3);
  Eigen::VectorXd positionJ5(runTime3);

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

  clock_t start, end;
  double result;

  start = clock(); //시간 측정 시작

  raisim::RaisimServer server(&world);
  server.launchServer();

  world.setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001); //:) material setting
  //world.exportToXml(binaryPath.getDirectory(), "exportedWorld.xml");

  /// save joint variables to txt
  ofstream fout("joint.txt", ios_base::out);

  Eigen::VectorXd jointNominalConfig(arm1->getGeneralizedCoordinateDim());
  jointNominalConfig << 1, 0, 0, 0, 0;
  arm1->setGeneralizedForce(jointNominalConfig);

  for (int i = 0; i < runTime; i++) {

      std::this_thread::sleep_for(std::chrono::microseconds(1));

      if (i == 15000) {
          jointNominalConfig << 0, 0, 0, 0, 0;
          arm1->setGeneralizedForce(jointNominalConfig);
      }

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
      
      positionJ1[i] = footPosition1[0];
      positionJ1[runTime + i] = footPosition1[1];
      positionJ1[runTime2 + i] = footPosition1[2];

      positionJ2[i] = footPosition2[0];
      positionJ2[runTime + i] = footPosition2[1];
      positionJ2[runTime2 + i] = footPosition2[2];

      positionJ3[i] = footPosition3[0];
      positionJ3[runTime + i] = footPosition3[1];
      positionJ3[runTime2 + i] = footPosition3[2];

      positionJ4[i] = footPosition4[0];
      positionJ4[runTime + i] = footPosition4[1];
      positionJ4[runTime2 + i] = footPosition4[2];

      positionJ5[i] = footPosition5[0];
      positionJ5[runTime + i] = footPosition5[1];
      positionJ5[runTime2 + i] = footPosition5[2];
      
      server.integrateWorldThreadSafe(); //:) 서버 유지

  }

  fout << positionJ1 << endl;
  fout << positionJ2 << endl;
  fout << positionJ3 << endl;
  fout << positionJ4 << endl;
  fout << positionJ5 << endl;
  fout.close();

  server.killServer();
  end = clock(); //시간 측정 끝
  result = (double)(end - start);
  printf("%f", result);
  return 0;
}