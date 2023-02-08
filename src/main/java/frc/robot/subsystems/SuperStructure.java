package frc.robot.subsystems;

import java.util.Vector;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

import frc.robot.Enums.ArmTargets;
import frc.robot.simulation.SimMechanism;



public class SuperStructure {
  private static SuperStructure mInstance;

  
  public static synchronized SuperStructure getInstance() {
    if (mInstance == null) {
      mInstance = new SuperStructure();
    }
    return mInstance;
  }

  // Reference to the Arm
  private Arm mArm = Arm.getInstance();
  private SimMechanism mSim = SimMechanism.getInstance();

  private boolean mRealRobot;

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

  private SuperStructure() {
    mRealRobot = false;

    // Default the Arm Targets to null
    mCurrentTargetPosition = null;
    mPreviousTargetPosition = null;
    mArmTargetPosition = null;
    mOverridingExtension = false;
    mOverridingAngle = false;

  }

  // Set if Sim or Real
  public void setRealRobot(boolean real)
  {
    mRealRobot = real;
  }

  private boolean evaluateArmAngle ()
  {
    boolean result = false;
    if(mRealRobot)
    {
      // real
      result = mArm.isArmRotationAtAngle(mArmTargetPosition.armAngle);
    }else {
      // sim
      result = (mSim.getArmAngle() == mArmTargetPosition.armAngle);
    }
    return result;
  }

  private boolean evaluteExtensionSafety()
  {
    boolean result = false;
    if(mRealRobot)
    {
      // real
      result = mArm.isArmExtensionAtPosition(mExtensionSafePosition, mExtensionPositionDeadband);
    } else {
      if((mSim.getArmExtension() >= (mExtensionSafePosition - mExtensionPositionDeadband))
        && (mSim.getArmExtension() <= (mExtensionSafePosition + mExtensionPositionDeadband))
      )
      {
        result = true;
      }
    }
    return result;
  }

  public void update() {
    // initial state 
    if(null == mCurrentTargetPosition)
    {
      mCurrentTargetPosition = mArmTargetPosition;
    }

    // is Extension at Safe Position
    /*
    boolean isExtensionSafe = mArm.isArmExtensionAtPosition(mExtensionSafePosition, mExtensionPositionDeadband);
    boolean isAtTargetArmAngle = mArm.isArmRotationAtAngle(mArmTargetPosition.armAngle);
    */
    boolean isExtensionSafe = evaluteExtensionSafety();
    boolean isAtTargetArmAngle = evaluateArmAngle();
    boolean isAtTargetOverallPosition = (mCurrentTargetPosition == mArmTargetPosition);

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
      } 
    }

    // Check if Arm made it to its target angle
    // We don't care about extension, that will sort itself out
    //if(mArm.isArmRotationAtAngle(mArmTargetPosition.armAngle))
    if(evaluateArmAngle())
    {
      mCurrentTargetPosition = mArmTargetPosition;
      mOverridingExtension = false;
      mOverridingAngle = false;
    }

    // Set Arm Angle and Arm Extension
    mArm.setArmAngle(mOverridingAngle ? mArmOverride: mCurrentTargetPosition.armAngle);
    mArm.setArmExtension(mOverridingExtension ? mExtensionOverride : mCurrentTargetPosition.armExtend);
  }


  public void setTargetArmPosition(ArmTargets targetPosition)
  {
    mArmTargetPosition = targetPosition;
  }

  

  public void updateSmartDashBoard()
  {

  }
}
