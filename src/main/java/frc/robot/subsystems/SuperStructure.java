package frc.robot.subsystems;

import java.util.Vector;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

import frc.robot.Enums.ArmTargets;



public class SuperStructure {

  // Reference to the Arm
  private Arm mArm = Arm.getInstance();

  private ArmTargets mCurrentTargetPosition;
  private ArmTargets mPreviousTargetPosition;
  private ArmTargets mArmTargetPosition;

  private boolean mOverridingExtension;
  private boolean mOverridingAngle;

  // Position the extension must be at 
  private double mExtensionSafePosition = Constants.Arm.MinExtensionPosition;

  // Deadband of the extension position
  private double mExtensionPositionDeadband = 20000;
  private double mExtensionOverride;
  private double mArmOverride;

  public SuperStructure() {
    // Default the Arm Targets to null
    mCurrentTargetPosition = null;
    mPreviousTargetPosition = null;
    mArmTargetPosition = null;
    mOverridingExtension = false;
    mOverridingAngle = false;

  }

  public void update() {
    // is Extension at Safe Position
    boolean isExtensionSafe = mArm.isArmExtensionAtPosition(mExtensionSafePosition, mExtensionPositionDeadband);
    boolean isAtTargetArmAngle = mArm.isArmRotationAtAngle(mArmTargetPosition.armAngle);
    boolean isAtTargetOverallPosition = (mCurrentTargetPosition != mArmTargetPosition);

    // Arm Target does not match current position
    if(!isAtTargetOverallPosition)
    {
      // Arm wants to move 
      // if currently the arm is extended, it has to be pulled in for safe travel
      if(!isExtensionSafe)
      {
        // Extension is not Safe, we can not move until the extension is pulled in
        mOverridingExtension = true;
        mExtensionOverride = mExtensionSafePosition;
      } else if(isExtensionSafe && !isAtTargetArmAngle)
      {
        // Extension is in safe position, continue to override to keep it safe
        // arm however is not at its target 
        // we are free to move the arm
        mOverridingAngle = true;
        mArmOverride = mArmTargetPosition.armAngle;       
      } else if(isExtensionSafe && isAtTargetArmAngle)
      {
        // Arm is good with angle
        // Now allow extension to go 
        mExtensionOverride = mArmTargetPosition.armExtend;
      }
    }

    // Check if Arm made it to both targets 
    //boolean isArm

    // Set Arm Angle and Arm Extension
    mArm.setArmAngle(mOverridingAngle ? mArmOverride: mCurrentTargetPosition.armAngle);
    mArm.setArmExtension(mOverridingExtension ? mExtensionOverride : mCurrentTargetPosition.armExtend);
  }


  public void setTargetArmPosition(ArmTargets targetPosition)
  {

  }

  

  public void updateSmartDashBoard()
  {

  }
}
