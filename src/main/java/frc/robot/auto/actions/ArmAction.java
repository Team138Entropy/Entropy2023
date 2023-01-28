package frc.robot.auto.actions;

import frc.robot.Enums.ArmTargets;
import frc.robot.subsystems.Arm;

public class ArmAction implements Action{
    private final Arm mArm = Arm.getInstance();
    private final double mTargetAngle;
    private final double mTargetExtend;
  
    public ArmAction(ArmTargets target){
      mTargetAngle = target.armAngle;
      mTargetExtend = target.armExtend;
    }
  
    @Override
    public void start() {
  
    }
  
    @Override
    public void update(){
      mArm.setArmAngle(mTargetAngle);
      mArm.setArmExtension(mTargetExtend);
    }
  
    @Override
    public boolean isFinished() {

        return Math.abs(mArm.getArmAngle() - mTargetAngle) < 10 && Math.abs(mArm.getArmExtension() - mTargetExtend) < 10;
    }
  
    @Override
    public void done() {

    }
}