package frc.robot.subsystems;

import java.io.IOException;
import java.util.ListIterator;
import java.util.Vector;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.util.physics.ArmConstraint;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Enums.ArmControlType;
import frc.robot.Enums.ArmRotationSpeed;
import frc.robot.Enums.ArmTargets;
import frc.robot.simulation.SimMechanism;

public class Superstructure {
  private static Superstructure mInstance;  
  public static synchronized Superstructure getInstance() {
    if (mInstance == null) {
      mInstance = new Superstructure();
    }
    return mInstance;
  }

  // Reference to the Arm
  private Arm mArm = Arm.getInstance();
  private Grasper mGrasper = Grasper.getInstance();
  private SimMechanism mSim = SimMechanism.getInstance();

  private boolean mRealRobot;

  private ArmTargets mCurrentTargetPosition;
  private ArmTargets mPreviousTargetPosition;
  private ArmTargets mArmTargetPosition;

  // Arm Control Style (Default to Simple)
  private ArmControlType mArmControlType = ArmControlType.Simple;

  private boolean mOverridingExtension;
  private boolean mOverridingAngle;

  // Position the extension must be at 
  private double mExtensionSafePosition = Constants.Arm.MinExtensionPosition;

  // Deadband of the extension position
  private double mExtensionPositionDeadband = 20000;
  private double mExtensionOverride;
  private double mArmOverride;

  // Option to Disable Arm Safety
  private boolean mDisableArmSafety;


  private Superstructure() {
    mRealRobot = false;

    // Default the Arm Targets to null
    mCurrentTargetPosition = null;
    mPreviousTargetPosition = null;
    mArmTargetPosition = null;
    mOverridingExtension = false;
    mOverridingAngle = false;
    mDisableArmSafety = false;
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

  private double getArmAngle()
  {
    double result = 0;
    if(mRealRobot)
    {
      result = mArm.getArmAngle();
    }else {
      result = mSim.getArmAngle();
    }
    return result;
  }

  private boolean evaluteExtension()
  {
    boolean result = false;
    if(mRealRobot)
    {
      // real
      result = mArm.isArmExtensionAtPosition(mArmTargetPosition.armExtend, mExtensionPositionDeadband);
    } else {
      if((mSim.getArmExtension() >= (mArmTargetPosition.armExtend - mExtensionPositionDeadband))
        && (mSim.getArmExtension() <= (mArmTargetPosition.armExtend + mExtensionPositionDeadband))
      )
      {
        result = true;
      }
    }
    return result;
  }

  private double getExtensionPosition()
  {
    double result = 0;
    if(mRealRobot)
    {
      result = mArm.getArmExtensionPosition();
    }else {
      result = mSim.getArmExtension();
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

    // Proceed Based on Arm Extension Type 
    switch(mArmControlType)
    {
      case Simple:
        processSimpleArmControl();
      break;
      case Advanced:
        processAdvancedArmControl();
      break;
      default:
      break;
    }
  }

  // Process Simple Arm Control
  private void processSimpleArmControl()
  {

    // is Extension at Safe Position
    /*
    boolean isExtensionSafe = mArm.isArmExtensionAtPosition(mExtensionSafePosition, mExtensionPositionDeadband);
    boolean isAtTargetArmAngle = mArm.isArmRotationAtAngle(mArmTargetPosition.armAngle);
    */
    boolean isExtensionSafe = evaluteExtensionSafety();
    boolean isAtTargetArmAngle = evaluateArmAngle();
    boolean isAtTargetOverallPosition = (mCurrentTargetPosition == mArmTargetPosition);
    if(mCurrentTargetPosition == ArmTargets.TOP_SCORING_FRONT && mArmTargetPosition == ArmTargets.TOP_SCORING_FRONT_SKILLSHOT){
      isExtensionSafe = true;

    }else if(mCurrentTargetPosition == ArmTargets.TOP_SCORING_FRONT_SKILLSHOT && mArmTargetPosition == ArmTargets.TOP_SCORING_FRONT){
      isExtensionSafe = true;

    }else if(mCurrentTargetPosition == ArmTargets.TOP_SCORING_FRONT && mArmTargetPosition == ArmTargets.TOP_SCORING_FRONT_CUBE){
      isExtensionSafe = true;

    }else if(mCurrentTargetPosition == ArmTargets.TOP_SCORING_FRONT_CUBE && mArmTargetPosition == ArmTargets.TOP_SCORING_FRONT){
      isExtensionSafe = true;

    }else if(mCurrentTargetPosition == ArmTargets.TOP_SCORING_FRONT_CUBE && mArmTargetPosition == ArmTargets.TOP_SCORING_FRONT_SKILLSHOT){
      isExtensionSafe = true;

    }else if(mCurrentTargetPosition == ArmTargets.TOP_SCORING_FRONT_SKILLSHOT && mArmTargetPosition == ArmTargets.TOP_SCORING_FRONT_CUBE){
      isExtensionSafe = true;

    }


    // Set Arm Speed - Don't think we should do this anymore!
    //mArm.setArmSpeeds(getRotationSpeed(mArmTargetPosition, mCurrentTargetPosition));

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
        if(mCurrentTargetPosition.armAngle > 90 && mArmTargetPosition.armAngle < 90 || mCurrentTargetPosition.armAngle < 90 && mArmTargetPosition.armAngle > 90){
          mGrasper.setGrasperClosed();
        }
        
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

  // Process Advanced Arm Control
  private void processAdvancedArmControl() 
  {
    // if not at arm target
    if(!isAtTarget())
    {
      // Get Arm Setpoints
      Pair<Double, Double> armSetPoints = getArmSetPoints(
        getArmAngle(), getExtensionPosition(), mArmTargetPosition);

      // Set Into Arm
      mArm.setArmAngle(armSetPoints.getFirst());
      mArm.setArmExtension(armSetPoints.getSecond());
    }else {
      // At Target Arm Position
      mCurrentTargetPosition = mArmTargetPosition;
      mArm.setArmAngle(mCurrentTargetPosition.armAngle);
      mArm.setArmExtension(mCurrentTargetPosition.armExtend);
    }
  }

  public void setTargetArmPosition(ArmTargets targetPosition)
  {
    mArmTargetPosition = targetPosition;
  }

  public ArmTargets getTargetArmPosition()
  {
    return mArmTargetPosition;
  }

  // Return True if Both are staisifed
  public boolean isAtTarget() {
    return evaluateArmAngle() && evaluteExtension();
  }

  // Changes the Arm Rotation Speed Based on what is needed
  public ArmRotationSpeed getRotationSpeed(ArmTargets targetPosition, ArmTargets currentPosition)
  {
    ArmRotationSpeed result = ArmRotationSpeed.DEFAULT;
    if(
      null != targetPosition &&
      null != currentPosition &&
      targetPosition != currentPosition
    )
    {
      /*
      // Greater than 90 is towards the backside, less than 90 is toward the front
      if(targetPosition.armAngle > 90 && currentPosition.armAngle < 90)
      {
        // Target Position is greater than 90, heading to backside
        result = ArmRotationSpeed.OVER_TOP_BACKWARDS;
      } else if(targetPosition.armAngle < 90 && currentPosition.armAngle > 90)
      {
        // Target Position is less than 90, heading to frontside
        result = ArmRotationSpeed.OVER_TOP_FORWARDS;
      }
      */
    }
    return result;
  }  

  // Disable Arm Safety
  public void setDisableArmSafety(boolean value)
  {
    mDisableArmSafety = value;
  }

  // Get if Arm Safety is Disable 
  public boolean getDisableArmSafety()
  {
    return mDisableArmSafety;
  }

  // Is the Arm at the front? Or is it going to the front?
  //"I think it should be isComprimisedCGStateThatJoeWantedImplementedAndWasImplementedByTheChefAKAGeorgeBOhAndAlsoGeorgeDrinksDrPepperGoTeam138Poggers()" -Avery
  public boolean isCGCompromised()
  {
    boolean result = false;
    if(mCurrentTargetPosition != null){
      if(mCurrentTargetPosition.armAngle <= (ArmTargets.TOP_SCORING_FRONT.armAngle + 15) 
        && mCurrentTargetPosition.armAngle >= (ArmTargets.TOP_SCORING_FRONT.armAngle - 15)){
        result = true;
      }
    }
    
    if(mArmTargetPosition != null){
      if(mArmTargetPosition.armAngle <= (ArmTargets.TOP_SCORING_FRONT.armAngle + 15) 
        && mArmTargetPosition.armAngle >= (ArmTargets.TOP_SCORING_FRONT.armAngle - 15)){
        result = true;
      }

      // If Arm is going back to the front you don't have to worry
      if(mArmTargetPosition.armAngle > 90){
        result = false;
      }
    }

    return result;
  }


  // Returns Angle and Extenion based on current positions and constraints
  public Pair<Double, Double> getArmSetPoints(double currentAngle, double currentExtension, ArmTargets targetPosition)
  {
    // Default these values to the Target Values, but they may get changed
    double OutputExtension = targetPosition.armExtend;
    double OutputAngle = targetPosition.armAngle;

    // direction 
    // Back Values are Greater than Front values
    // 0 is Frontside of Robot, 270 is Backside
    // List will be ordered from least to greatest, meaning rotating forward is backwards through list
    boolean rotatingForward = (currentAngle > targetPosition.armAngle);
    if(rotatingForward){
      System.out.println("Rotating Forward!");
    }

    // Is Rotation even required? 
    boolean isAtTargetArmAngle = evaluateArmAngle();

    // Rotation Required 
    if(!isAtTargetArmAngle)
    {
      // Rotating forward, iterate further
      final int listSize = Constants.Arm.ArmConstraints.size();
      int startIndex = (rotatingForward ? listSize : 0);
      ListIterator<ArmConstraint> listIterator = Constants.Arm.ArmConstraints.listIterator(startIndex);

      // Design of this is to get the arm angle as close as possible to the target arm angle as possible
      // Arm is Backside Home (~225, Retracted Full) and wants to go to Score Front High (~0, Full Extension)
      //      Need to Understand the Constraints in the way, even though the arm may not be immediately constrainted extension wise, 
      //      it will be on its path 

      // Goal is to maximize extension staying in constraints
      
      // TODO: Constraint class for now just enforces max extension..which is all we need it for
      

      // Advance Either Forward or Backwards through list
      while((rotatingForward ? listIterator.hasPrevious() : listIterator.hasNext()))
      {
        ArmConstraint currentConstraint = rotatingForward ? 
                                      (ArmConstraint) listIterator.previous() : 
                                      (ArmConstraint) listIterator.next();

        // Rotating Forward - Arm Angle is Decreasing
        // Rotating Backward - Arm Angle is Increasing
        
        // Is this an arm angle the constraint system cares about?
        // Determine if its an angle within the path to the target where the arm cuttently is
        boolean isRelevantConstraint = false;
        isRelevantConstraint |= (rotatingForward && (currentAngle >= currentConstraint.Angle) && (currentConstraint.Angle >= targetPosition.armAngle));
        isRelevantConstraint |= (!rotatingForward && (currentAngle <= currentConstraint.Angle) && (currentConstraint.Angle <= targetPosition.armAngle));

        // 
        if(isRelevantConstraint){
          // Rotating Forward, This Angle Constraint is a lesser angle meaning it is in the path 

          // evaluate if current extension is below or equal to this max height
          if(!isExtLessThanOrEqualMax(currentExtension, currentConstraint.Value))
          {
            // Current Extension is not less than the maximum constraint
            // Angle must be held here until this corrects
            OutputAngle = currentConstraint.Angle;

            // Set Current Extension Max
            //  If overall desire of extension is even less than max, set that instead
            OutputExtension = Math.min(currentConstraint.Value, mArmTargetPosition.armAngle);

            break;
          }

          // Evaluate the Current Constraint
          // Because it is a constraint, must match it
          if(OutputExtension > currentConstraint.Value)
          {
            OutputExtension = currentConstraint.Value;
          }
        }                
      }
    }

  

    // Debug Logging (if not at target)
    if(!isAtTarget())
    {
      System.out.println("ArmSet: Ang: " + OutputAngle +" Ext: " + OutputExtension 
        + "  Target Ang: " + mArmTargetPosition.armAngle + " Target Ext: " + mArmTargetPosition.armExtend
        + "  Cur Ang: " + currentAngle + "  Cur Ext: " + currentExtension
      );

    }


    return new Pair<Double, Double>(OutputAngle, OutputExtension);
  }

  private boolean isExtLessThanOrEqualMax(double currExtPos, double maxValue)
  {
    boolean result = false;
    if(mRealRobot)
    {
      // real
      // If current extension position is less than or requal to the max value
      result = (currExtPos <= (maxValue + mExtensionPositionDeadband));
    } else {
      result = (mSim.getArmExtension() <= (maxValue + mExtensionPositionDeadband));
    }
    return result;
  }


  // Set the Arm Control Style
  public void setArmControlType(ArmControlType ct)
  {
    mArmControlType = ct;
  }

  // Get Current Arm Control Style
  public ArmControlType getArmControlType()
  {
    return mArmControlType;
  }

  public void updateSmartDashBoard()
  {
    final String key = "Superstructure/";
    SmartDashboard.putBoolean(key + "Extension Overriding", mOverridingExtension);
    SmartDashboard.putBoolean(key + "Angle Overriding", mOverridingAngle);
    SmartDashboard.putBoolean(key + "CG Compromised", isCGCompromised());

  }
}
