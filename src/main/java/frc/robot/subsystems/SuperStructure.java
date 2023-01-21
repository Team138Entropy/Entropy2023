package frc.robot.subsystems;

import frc.robot.Enums.ArmTargets;

public abstract class SuperStructure extends Subsystem{

  public double armAngle = 45;

  public double armExtension = 0;




  public SuperStructure() {
  }

  /*
  Joes Notes on what SuperStructure needs to do
   - needs to be a singelton like Drive or Arm or Grasper etc. that way we can call 'getInstance' to get it in Robot.java
   - needs to have an update function that we will call constantly from RobotLoop

   superstructure is going to consider the angle of the arm, and understand what the extension has to do

   lets say that the arm works the same way as last year and 45 is completely horizontal on the front side, 135 is completely horizontal on the back
   if my arm is at 135 and i am saying i want to go to 45, i know i'm going to go over the top. MEANING my extension is going to have to be constrained
   to a maximum height. We need to come up with some sort of equation of if were going over the top... i'm not sure yet, basically we can determine we are going over the top
   if we are going to pass through the values of 75-100. Not sure how to determine yet. dont worry about it

   from RobotLoop in RObot.java we need to constantly tell superstructure what our Arm Target is
   in superstructure we will check our Arm and get the Extension Length and the Arm Rotation. since we don't have those subsystems in master yet, just use placeholder variables
   */

  public void update() {
    //placeholder update fucntions
    //armTargetAngle = Arm.getAngleTarget();
    //armExtension = Arm.getExtension
  }

  

  public abstract void updateSmartDashBoard();
}
