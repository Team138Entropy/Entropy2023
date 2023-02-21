package frc.robot.auto.actions;

import frc.robot.Enums.*;
import frc.robot.subsystems.Arm;


public class ScoreAction implements Action {
    private final Arm mArm = Arm.getInstance();

    double mArmAngle;
    double mArmExtension;

    public ScoreAction(ArmTargets target, TargetedObject gameObject){
        mArmAngle = target.armAngle;
        mArmExtension = target.armExtend;
        
      }
    
      @Override
      public void start() {
        mArm.setArmAngle(mArmAngle);
        mArm.setArmExtension(mArmExtension);
      }
    
      @Override
      public void update(){
        if(Math.abs(mArm.getArmAngle() - mArmAngle) < 10 && Math.abs(mArm.getArmExtension() - mArmExtension) < 10){
            
        }
        
      }
    
      @Override
      public boolean isFinished() {
        return true;
      }
    
      @Override
      public void done() {
  
      }
}
