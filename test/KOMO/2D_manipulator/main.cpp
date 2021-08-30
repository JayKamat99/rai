#include <KOMO/komo.h>
#include <Kin/viewer.h>

void goToBall(){
  rai::Configuration C("2D.g"); // Load the configuration
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., 10, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"gripper", "ball"}, OT_sos, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);  //small velocity in that time frame
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
  komo.add_collision(true);

  komo.animateOptimization = 1;
  // komo.initWithWaypoints({{1,1},{2,2}} , 2, false); //false-> it's linear interpolation and not sine
  
  // for (int i=0; i<5; ++i){
  //   komo.optimize();
  //   komo.plotTrajectory();
  //   komo.checkGradients();
  //   std::cout << i <<"\n";
  //   rai::ConfigurationViewer V;
  //   V.setPath(C, komo.x, "result", false);
  //   while(V.playVideo());
  // }
  komo.optimize();
  komo.plotTrajectory();
  komo.checkGradients();
  rai::ConfigurationViewer V;
  V.setPath(C, komo.x, "result", false);
  while(V.playVideo());
}

void pickAndPlace(bool keyframesOnly){
  rai::Configuration C("2D.g");
  rai::ConfigurationViewer V;
  V.setConfiguration(C, "initial model", false); //Displays the initial model.

  KOMO komo;

  komo.setModel(C, true); // false-> doesn't compute collissions
  if(!keyframesOnly){
    komo.setTiming(2.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

    //grasp
  komo.addSwitch_stable(1., 2., "world", "gripper", "ball");
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "ball"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "ball"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }


  //place
  komo.addSwitch_stable(2., -1., "gripper", "goal", "ball", true);
  komo.addObjective({2.}, FS_positionDiff, {"ball", "goal"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }

  komo.verbose = 4;
  komo.animateOptimization=1;
  // komo.initWithWaypoints({{1,1},{2,2},{3,3},{3,3}} , 2, false);
  komo.optimize();

  komo.view(true, "optimized motion");
  for(uint i=0;i<2;i++) komo.view_play(true);
}


int main(int argc,char** argv){
  rai::initCmdLine(argc,argv); // This logs the inputs..

  goToBall();
  // pickAndPlace(false);

  return 0;
}