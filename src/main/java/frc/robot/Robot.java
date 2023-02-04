package frc.robot;

import org.photonvision.SimVisionTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Enums.*;
import frc.robot.OI.OperatorInterface;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.TrajectoryFollower;
import frc.robot.auto.TrajectoryGeneratorHelper;
import frc.robot.auto.modes.*;
import frc.robot.simulation.SimMechanism;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive.DriveStyle;
import frc.robot.util.drivers.Pigeon;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.vision.AutoPilot;
import frc.robot.vision.chargingStationAutoPilot;
import frc.robot.vision.photonVision;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();


  // Controllers Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  // Simulation Mechanism
  private final SimMechanism mSimMechanism = SimMechanism.getInstance();

  // Trajectory Follower
  private final TrajectoryFollower mTrajectoryFollower = TrajectoryFollower.getInstance();

  // PhotonVision
  private final photonVision mPhotonVision = photonVision.getInstance();

  // Vision Driver
  private final AutoPilot mAutoPilot = AutoPilot.getInstance();

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  
  // Subsystems
  private final Drive mDrive = Drive.getInstance();
  private final Arm mArm = Arm.getInstance();
  private final Grasper mGrasper = Grasper.getInstance();

  private final chargingStationAutoPilot mChargingStationAutoPilot = chargingStationAutoPilot.getInstance();

  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;
  private AutoModeBase mAutoModeBase = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  private SendableChooser<TargetedPositions> mTargetedPositionChooser;


  // Field Object
  private final Field2d mField = new Field2d();

  // Pigeon Reference
  private final Pigeon mPigeon = Pigeon.getInstance();

  // Real Robot Indicator
  private final boolean mRealRobot = Robot.isReal();

  // Reference to the Power Distrubtion Panel
  private final PowerDistribution mPowerPanel = new PowerDistribution(Constants.Talons.PowerDistribution.pdpCan, 
                                                                                                  ModuleType.kRev);

  // DIO Based Analog Input
  private final AnalogInput mPressureSensor = new AnalogInput(0);

  // Various Variables
  int mRumbleTimer = 0;

  // Test Controls
  private boolean mJogMode = true;
  private boolean mPositionMode = false;
  private boolean mGrasperOpen = true;

  private double mTestTargetPositionDegrees = ArmTargets.HOME_BACKSIDE.armAngle;
  private double mTestArmExtension = 0;


  // Arm Target
  public ArmTargets mCurrentArmTarget = ArmTargets.HOME_BACKSIDE;
  public ArmControlType mArmControlType = ArmControlType.Simple;

  public int mManualTargetOffset = 0;

  public int mManualExtendOffset = 0;

  public boolean mBalanceMode = false;

  // Position the Auto Pilot System Wants to Drive to
  public TargetedPositions mTargetedPosition = TargetedPositions.NONE;
  public TargetedObject mCurrentTargetedObject = TargetedObject.CONE;


  /**
   * On Robot Startup
   */
  @Override
  public void robotInit() {

    // Temp Target Position Chooser - Eventually this will be by button!
    mTargetedPositionChooser = new SendableChooser<TargetedPositions>();
    mTargetedPositionChooser.setDefaultOption("GRID_BOTTOM_1", TargetedPositions.GRID_BOTTOM_1);
    mTargetedPositionChooser.addOption("NONE", TargetedPositions.NONE);
    mTargetedPositionChooser.addOption("GRID_BOTTOM_1", TargetedPositions.GRID_BOTTOM_1);
    mTargetedPositionChooser.addOption("GRID_BOTTOM_2", TargetedPositions.GRID_BOTTOM_2);
    mTargetedPositionChooser.addOption("GRID_BOTTOM_3", TargetedPositions.GRID_BOTTOM_3);
    mTargetedPositionChooser.addOption("GRID_MIDDLE_1", TargetedPositions.GRID_MIDDLE_1);
    mTargetedPositionChooser.addOption("GRID_MIDDLE_2", TargetedPositions.GRID_MIDDLE_2);
    mTargetedPositionChooser.addOption("GRID_MIDDLE_3", TargetedPositions.GRID_MIDDLE_3);
    mTargetedPositionChooser.addOption("GRID_TOP_1", TargetedPositions.GRID_TOP_1);
    mTargetedPositionChooser.addOption("GRID_TOP_2", TargetedPositions.GRID_TOP_2);
    mTargetedPositionChooser.addOption("GRID_TOP_3", TargetedPositions.GRID_TOP_3);
    mTargetedPositionChooser.addOption("RED_SUBSTATION_LEFT", TargetedPositions.RED_SUBSTATION_LEFT);
    mTargetedPositionChooser.addOption("RED_SUBSTATION_RIGHT", TargetedPositions.RED_SUBSTATION_RIGHT);
    mTargetedPositionChooser.addOption("BLUE_SUBSTATION_LEFT", TargetedPositions.BLUE_SUBSTATION_LEFT);
    mTargetedPositionChooser.addOption("BLUE_SUBSTATION_RIGHT", TargetedPositions.BLUE_SUBSTATION_RIGHT);
    SmartDashboard.putData(mTargetedPositionChooser);

    // Start Datalog Manager
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());  
    
    // Real or Simulation Robot
    mRobotState.setRealRobot(mRealRobot);
    mDrive.setRealRobot(mRealRobot);

    // populate autonomous list
    populateAutonomousModes();

    // generate generic auto modes to load into JIT
    TrajectoryGeneratorHelper.generateExampleTrajectories();

    // Configure Arm's Maximum and Minimum Rotation Settings
    mArm.setArmRotationMinimum(Enums.ArmTargets.INTAKE_GROUND_FRONT.armAngle);
    mArm.setArmRotationMaximum(Enums.ArmTargets.START.armAngle);

    // reset odometry

    // Reset Drive Sensors

    // Controllable Panel
    mPowerPanel.setSwitchableChannel(true);
    mArm.zeroSensors();
  }

  /**
   * Robot Periodic - Called Constantly throughout Robot Operation
   */
  @Override
  public void robotPeriodic() {
    // Update Robotstate
    mRobotState.update();

    // Update Auto Pilot
    mAutoPilot.setRobotPose(mRobotState.getPose());

    // Update Smartdashboard Overall and Subsystems
    updateSmartdashboard();

    mTargetedPosition = mTargetedPositionChooser.getSelected();
  }

  private void updateSmartdashboard()
  {
    SmartDashboard.putData("Field", mField);
    SmartDashboard.putString("Robot Pose", mField.getRobotPose().toString());
    SmartDashboard.putNumber("Pigeon Degrees", mPigeon.getYaw().getDegrees());
    SmartDashboard.putNumber("Pigeon Radians", mPigeon.getYaw().getRadians());
    SmartDashboard.putString("Target Position", mTargetedPosition.toString());
    SmartDashboard.putString("Target Arm Position", mCurrentArmTarget.toString());
    SmartDashboard.putNumber("Target Arm Angle", mCurrentArmTarget.armAngle);
    SmartDashboard.putNumber("Target Arm Extension", mCurrentArmTarget.armExtend);
    SmartDashboard.putBoolean("Automatic Mode", mPositionMode);
    SmartDashboard.putNumber("rotate offset", mManualTargetOffset);
    SmartDashboard.putNumber("extend offset", mManualExtendOffset);
    
    //formula to convert to PSI
    SmartDashboard.putNumber("pressure sensor", 250.0 * mPressureSensor.getVoltage() / 5.0 - 25.0);
    SmartDashboard.putData(mPowerPanel);

    // Controls
    final String controlsKey = "Controls/";
    SmartDashboard.putNumber(controlsKey + "Drive Throttle", mOperatorInterface.getDriveThrottle());
    SmartDashboard.putNumber(controlsKey + "Drive Turn", mOperatorInterface.getDriveTurn());
    SmartDashboard.putString(controlsKey + "ArmControlMode", mArmControlType.toString());
    SmartDashboard.putBoolean(controlsKey + "Jog Mode", mJogMode);

    // Auto Mode Executor
    if(null != mAutoModeExecutor) mAutoModeExecutor.updateSmartDashboard();

    // Auto Pilot Driver
    mAutoPilot.updateSmartDashBoard();

    // PhotonVision Smartdashboard
    mPhotonVision.updateSmartDashboard();

    // Trajectory Follower 
    mTrajectoryFollower.updateSmartdashboard();

    // RobotState
    mRobotState.updateSmartdashboard();

    // Sim Mechanism 
    mSimMechanism.updateSmartDashboard(); // Sim Only

    // Iterates each Subsystem 
    mSubsystemManager.updateSmartdashboard();

    mGrasper.updateSmartDashBoard();


  }

  // Fill Autonomous Modes List
  private void populateAutonomousModes(){
    // Auto Mode
    mAutoModes = new SendableChooser<AutoModeBase>();
    //mAutoModes.setDefaultOption("Nothing", new DoNothingMode());
    mAutoModes.setDefaultOption("Test Swerve Mode", new SwerveTestAutoMode() );
    SmartDashboard.putData(mAutoModes);
  }

  /***
   * Start of the Autonomous Mode
   */
  @Override
  public void autonomousInit() {
    // Disable Operator Rumble
    mOperatorInterface.setOperatorRumble(false);

    // zero sensors (if not zero'ed prior on this powerup)
    mSubsystemManager.zeroSensorsIfFresh();

    // Get Selected AutoMode
    mAutoModeExecutor.setAutoMode(mAutoModes.getSelected());
    mAutoModeBase = mAutoModes.getSelected();
    mAutoModeBase.reset();

    // Close Grasper
    mGrasper.setGrasperClosed();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Autonomous is run through the AutoModeExecutor
    if(!mAutoModeBase.isDone()){
      mAutoModeBase.runner();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
    // disable operator rumble    
    mOperatorInterface.setOperatorRumble(false);
        
    // zero sensors (if not zero'ed prior on this powerup)
    mSubsystemManager.zeroSensorsIfFresh();
    
    // Disable Auto Thread (if running)
    if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
    }

    // Zero Drive Sensors
    mDrive.zeroSensors();

    // Close Grasper
    mGrasper.setGrasperClosed();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mOperatorInterface.setDriverRumble(mBalanceMode, mBalanceMode ? .2 : 0);
    // Main Robot Loop!
    RobotLoop();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Reset all auto mode state.
    if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
    }

    // create Auto Mode Executor
    mAutoModeExecutor = new AutoModeExecutor();
  }


  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
      // Activate rumble on op controller every second or so
      if (mRumbleTimer > 2000){ mOperatorInterface.setOperatorRumble(true); }
      if (mRumbleTimer > 2025){ mOperatorInterface.setOperatorRumble(false); mRumbleTimer = 0; }
      mRumbleTimer++;
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Close Grasper
    mGrasper.setGrasperClosed();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    mGrasper.setGrasperWheelIntake();

      //Allows the operator to swap freely between both test modes by pressing START
    if (mJogMode == true && mOperatorInterface.getModeSwitch()){
        mJogMode = false;
        mPositionMode = true;
      }
    else if (mPositionMode == true && mOperatorInterface.getModeSwitch()){
        mJogMode = true;
        mPositionMode = false;
      }

      if (mGrasperOpen == true && mOperatorInterface.getGrasperModeSwap()){
        mGrasperOpen = false;
      }
      else if (mGrasperOpen == false && mOperatorInterface.getGrasperModeSwap()){
        mGrasperOpen = true;
      }

      //Zero encoders on the arm using the right stick button
      if(mOperatorInterface.getArmEncoderZero()){
        mArm.zeroSensors();
        System.out.println("Arm set to zero");
      }

    //Manual Functions
    if (mJogMode == true){
      

      //Manual extension of the arm using RB/LB
      if (mOperatorInterface.getArmJogExtended()){
        mArm.setExtensionJog(0.4);
      } 
      else if (mOperatorInterface.getArmJogRetracted()){
        mArm.setExtensionJog(-0.4);
      }      
      else {
        mArm.setExtensionJog(0);
      }  


      //Manual rotation of the arm using Y/A
    if (mOperatorInterface.getArmRotateForward()){
      mArm.setShoulderJog(0.3);  
    } 
    else if (mOperatorInterface.getArmRotateBackward()) {
      mArm.setShoulderJog(-0.3);
    }
    else {
      mArm.setShoulderJog(0);
    }

     //Grasper Open/Close using RT
     if (mOperatorInterface.getGrasperModeSwap()){
      mGrasper.setGrasperClosed();
    }
    else if (mOperatorInterface.getGrasperWheelIntake()){
      mGrasper.setGrasperOpen();
    }
    }  
    
    //Automatic functions
    if (mPositionMode == true) {

            //preset arm positions
      /*
       if (mOperatorInterface.getArmJogForward()){
        mArm.setArmAngle(135);
      }
      else if (mOperatorInterface.getArmJogMidForward()){
        mArm.setArmAngle(115);
      }
      else if (mOperatorInterface.getArmJogMidBackward()){
        mArm.setArmAngle(65);
      }
      else if (mOperatorInterface.getArmJogBackward()){
        mArm.setArmAngle(45);
      }
      else {
        mArm.setArmAngle(90);
      }
       */

       //Use Y/A/X/B to increase/decrease the current arm angle by 1 or 5 degrees  
       mArm.setArmAngle(mTestTargetPositionDegrees);

       if (mOperatorInterface.getArmAnglePlusOne()){
         mTestTargetPositionDegrees += 1;
       }
       else if (mOperatorInterface.getArmAngleMinusOne()){
         mTestTargetPositionDegrees -= 1;
       }
       else if (mOperatorInterface.getArmAnglePlusFive()){
         mTestTargetPositionDegrees += 5;
       }
       else if (mOperatorInterface.getArmAngleMinusFive()){
         mTestTargetPositionDegrees -= 5;
       }
 
       //Use the d-pad to adjust the arm extension by an inch or 5 inches
       mArm.setArmExtension(mTestArmExtension);
       
       if (mOperatorInterface.setArmExtendedPlusOne()){
         mTestArmExtension += 1;
       }
       else if (mOperatorInterface.setArmExtendedMinusOne()){
         mTestArmExtension -= 1;
       }
       else if (mOperatorInterface.setArmExtendedPlusFive()){
         mTestArmExtension += 5;
       }
       else if (mOperatorInterface.setArmExtendedMinusFive()){
         mTestArmExtension -= 5;
       }      
 

      /*
      //preset arm positions
      if (mOperatorInterface.getArmJogForward()){
        mArm.setArmAngle(135);
      }
      else if (mOperatorInterface.getArmJogMidForward()){
        mArm.setArmAngle(115);
      }
      else if (mOperatorInterface.getArmJogMidBackward()){
        mArm.setArmAngle(65);
      }
      else if (mOperatorInterface.getArmJogBackward()){
        mArm.setArmAngle(45);
      }
      else {
        mArm.setArmAngle(90);
      }

      //Automatic closing of the grasper
      if (mGrasper.getBeamSensorBroken() == true){
        mGrasper.setGrasperClosed();
      }
      else if (mGrasperOpen == false && mOperatorInterface.getGrasperModeSwap()){
        mGrasper.setGrasperOpen();
      }
      
      //Automatic extension positions 
      if (mOperatorInterface.getArmExtended2()){
        mArm.setArmExtension(2);
      }
      else if (mOperatorInterface.getArmExtended4()){
        mArm.setArmExtension(4);
      }
      else if (mOperatorInterface.getArmExtended6()){
        mArm.setArmExtension(6);
      }
      else if (mOperatorInterface.getArmExtended0()){
        mArm.setArmExtension(0);
      }
      */

    }

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

    // Add April Tag Fields to Camera Sim System
    mPhotonVision.simVision.addVisionTargets(FieldConstants.aprilTagField);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    
    mDrive.updateDriveSim();
    // Update Pose on Virtual Field

    // Simulate Swerve Rotation
    double sRotation = mOperatorInterface.getSwerveRotation();
    mPigeon.rotateSimYaw(sRotation);

    // Arm
    // Current Targeted Arm into Mechanism Sim
    // This will set the actuall commanded 
    //mSimMechanism.SetArmAngle(mCurrentArmTarget.armAngle + mManualTargetOffset);
    mSimMechanism.SetArmAngle(mArm.getArmTargtedDegrees());
    mSimMechanism.SetArmLength(mCurrentArmTarget.armExtend + mManualExtendOffset);

    // Grasper - Open or Close
    if(mGrasper.getGrasperState() == Grasper.GrasperState.Open)
    {
      mSimMechanism.openGrasper();
    } else 
    {
      mSimMechanism.closeGrasper();
    }

    // Process Frame where the Robot currently is
    mPhotonVision.simVision.processFrame(mField.getRobotPose());  
  }

  /** Called Throughout Teleop Periodic */
  private void RobotLoop(){
    // Drive the Robot
    DriveLoop(mOperatorInterface.getDrivePrecisionSteer(), true);

    // Update Robot Field Position
    Pose2d robotPose = mDrive.getPose();
    mField.setRobotPose(robotPose);
    mRobotState.setRobotPose(robotPose);

    // Operator Arm Commands
    if(mOperatorInterface.getScoringCommand() != TargetedPositions.NONE){
      mTargetedPosition = mOperatorInterface.getScoringCommand();
    }
    
    if(mOperatorInterface.getArmTarget() != ArmTargets.NONE){
      mCurrentArmTarget = mOperatorInterface.getArmTarget();
      mManualExtendOffset = 0;
      mManualTargetOffset = 0;
    }

    if(mOperatorInterface.getManualArmExtendUp()){
      mManualExtendOffset += 1;
    }else if(mOperatorInterface.getManualArmExtendDown()){
      mManualExtendOffset -= 1;
    }

    if(mOperatorInterface.getManualArmRotateUp()){
      mManualTargetOffset += 1;
    }else if(mOperatorInterface.getManualArmRotateDown()){
      mManualTargetOffset -= 1;
    }

    // Constantly tell the Arm where to go
    if(ArmControlType.Simple == mArmControlType)
    {
      // Simple Arm Control Type (Do not rotate arm until extension is Retracted)
      // If you want to move the arm to a different rotation angle, arm extension must be retracted
      // This is still a todo! 
      mArm.setArmAngle(mCurrentArmTarget.armAngle + mManualTargetOffset);
      mArm.setArmExtension(mCurrentArmTarget.armExtend + mManualExtendOffset);
    }
    else if(ArmControlType.Advanced == mArmControlType)
    {
      // TODO: Allow Arm to move with extension out
      // This is more complicated.. we might not get to this.. or need it
    }
 

    // Grasper Functionality
    if(mOperatorInterface.getGrasperOpen()){
      mGrasper.setGrasperOpen();
    }else if (
      mOperatorInterface.getGrasperClosed() || mGrasper.getBeamSensorBroken()
    ){
      // Close Grasper by close press or by beam
      mGrasper.setGrasperClosed();
    }
    mGrasper.update();
  }

   /**
   * DriveLoop
   * precisionSteer - Tunes Down Throttle. Useful for precise movements
   * allowAutoSteer - Enables/Disables AutoSteering
   */
  private void DriveLoop(boolean precisionSteer, boolean allowAutoSteer){
    double driveThrottle = mOperatorInterface.getDriveThrottle()*-1;
    double driveTurn = mOperatorInterface.getDriveTurn();
    SmartDashboard.putNumber("Driver Throttle", driveThrottle);
    SmartDashboard.putNumber("Driver Trun", driveTurn);

    // precision steer (slow down throttle if left trigger is held)
   if(precisionSteer) driveThrottle *= .3;

    boolean wantsAutoSteer = mOperatorInterface.getDriveAutoSteer();
    if(mOperatorInterface.getBalanceMode() && mBalanceMode == false){
      mBalanceMode = true;
    }else if(mOperatorInterface.getBalanceMode() && mBalanceMode == true){
      mBalanceMode = false;
    }
    //wantsAutoSteer &= allowAutoSteer; //disable if autosteer isn't allowed
    SmartDashboard.putBoolean("Autosteer", wantsAutoSteer);

    // Get Target within the allowed Threshold
    //TargetInfo ti = mVisionManager.getSelectedTarget(Constants.Vision.kAllowedSecondsThreshold);
    //boolean validTargetInfo = (ti != null);
    boolean validTargetInfo = false;
    double errorAngle = 0;
    boolean validTarget = false;
    if(validTargetInfo){
      // Valid Target Packet
      //errorAngle = ti.getErrorAngle();
      //validTarget = ti.isValid();
    }
    SmartDashboard.putBoolean("Valid Target", validTarget);
    SmartDashboard.putNumber("Target Angle", errorAngle);

    // Procced based on Drive Style
    Drive.DriveStyle driveStyle = mDrive.getDriveStyle();
    if(Drive.DriveStyle.DIFFERENTIAL_DRIVE == driveStyle)
    {
      // Differential Drive
      mDrive.setDrive(driveThrottle, driveTurn, false);
    }
    else if(Drive.DriveStyle.SWERVE_DRIVE == driveStyle)
    {
      // Zero Gyro
      if(mOperatorInterface.getZeroGyro())
      {
        mDrive.zeroHeading();
      }

      // Swerve Brake
      //mDrive.setBrake(mOperatorInterface.getBrake());

      // Auto Pilot (and has Valid Vision Pose)
      if(wantsAutoSteer && mRobotState.getVisionEstimatedPoseValid())
      {
        // if not running, set up initial system
        if(!mAutoPilot.getRunning())
        {
          // test code
          Pose2d StartingPose = mField.getRobotPose();
          //aprilTags
          AprilTag targetedTag = FieldConstants.getAprilTag(1);
          //test code
          if(TargetedPositions.GRID_BOTTOM_1 == mTargetedPosition)
          {
            targetedTag = FieldConstants.getAprilTag(1);
          }
          else if(TargetedPositions.GRID_BOTTOM_2 == mTargetedPosition)
          {
            targetedTag = FieldConstants.getAprilTag(2);
          }
          else if(TargetedPositions.GRID_BOTTOM_3 == mTargetedPosition)
          {
            targetedTag = FieldConstants.getAprilTag(3);
          }
          Pose2d targetedTagPose = targetedTag.pose.toPose2d();

          Translation2d selectedXY = new Translation2d();

          Alliance color = DriverStation.getAlliance();

          TargetedPositions pos = mTargetedPosition;
          int index = pos.ordinal();

          if(color == Alliance.Blue){
            selectedXY = FieldConstants.Grids.blueFinalScorePosition[index-1];
          }else if(color == Alliance.Red){
            selectedXY = FieldConstants.Grids.redFinalScorePosition[index-1];
          }
          
          //
          //FieldConstants.Grids.redFinalScorePosition[]
          //FieldConstants.Grids.blueFinalScorePosition[]

          // 
          Pose2d targetedTagPoseWithRotation = new Pose2d(selectedXY, StartingPose.getRotation());
      
          // test code

          mAutoPilot.setStartingPose(StartingPose);
          mAutoPilot.setTargetPose(targetedTagPoseWithRotation);
        }

        // Constantly Feed the Vision Updated Pose

        // general vision driving update
        mAutoPilot.update();
      }
      else if(mBalanceMode)
      {
        if(mOperatorInterface.getFastBalance())
          mChargingStationAutoPilot.update(false, mOperatorInterface.getAutoPilotLeftStrafe(), mOperatorInterface.getAutoPilotRightStrafe());
        else if(mOperatorInterface.getSlowBalance()){
          mChargingStationAutoPilot.update(true, mOperatorInterface.getAutoPilotLeftStrafe(), mOperatorInterface.getAutoPilotRightStrafe());
        }else{
          mDrive.setBrake(true);
        }
      }
      else
      {
        // Normal Swerve Operation

        // Do Not AutoPilot Drive
        mAutoPilot.stop();
        
        // Swerve Snap to a Direction (Button Press Quickly Moves Robot)
        SwerveCardinal snapCardinal = mOperatorInterface.getSwerveSnap();
        if(SwerveCardinal.NONE != snapCardinal) // Snap Direction Detected!
        {
          mDrive.startSnap(snapCardinal.degrees);
        }

        // Swerve Quick Adjust
        SwerveQuickAdjust quickAdjust = mOperatorInterface.getSwerveQuickAdjust();
        if(SwerveQuickAdjust.NONE != quickAdjust)
        {
          mDrive.startQuickAdjust(quickAdjust.targetPose);
        }

        // Swerve Drive
        Translation2d sTrans = mOperatorInterface.getSwerveTranslation();
        if(precisionSteer) sTrans = sTrans.times(.5); // slow down the speed by 50%!
        double sRotation = mOperatorInterface.getSwerveRotation();
        mDrive.setSwerveDrive(sTrans, sRotation, true, true, precisionSteer);

        // Log Inputs
        SmartDashboard.putString("Swerve Input Translation", sTrans.toString());
        SmartDashboard.putNumber("Swerve Input Rotation", sRotation);
      }
    }

    // Update Odometry of Robot (only if real)
    if(mRealRobot) mDrive.updateOdometry();
  }

}
