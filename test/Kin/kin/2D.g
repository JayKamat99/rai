world{}

Base (world) {X=<t(0 0 0) d(90 0 1 0)> }
arm1 { shape:capsule, mass:1, size:[.5 .05] }
arm2 { shape:capsule, mass:1, size:[.5 .05] }

joint1 (Base arm1) { joint:hingeX, A:<t(0 0 0) d(0 1 0 0)>, Q:<d(30 1 0 0)>, B:<t(0 0 .25) >}
joint2 (arm1 arm2) { joint:hingeX, A:<t(0 0 .25) d(0 1 0 0)>, Q:<d(30 1 0 0)>, B:<t(0 0 .25) > }

gripper (arm2){
    shape:marker, size:[.5], color:[.9 .9 .5],
    Q:<t(0 0 .25)>  
}

