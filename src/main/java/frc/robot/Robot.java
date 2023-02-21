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
import frc.robot.util.TuneableNumber;
import frc.robot.util.drivers.Pigeon;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.vision.AutoPilot;
import frc.robot.vision.chargingStationAutoPilot;
import frc.robot.vision.photonVision;
import frc.robot.subsystems.Grasper.GrasperState;

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
  private final Superstructure mSuperStructure = Superstructure.getInstance();

  private final chargingStationAutoPilot mChargingStationAutoPilot = chargingStationAutoPilot.getInstance();

  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;
  private AutoModeBase mAutoModeBase = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  // Position Target Chooser
  private SendableChooser<TargetedPositions> mTargetedPositionChooser;

  // Arm Target Chooser 
  private SendableChooser<ArmTargets> mArmTargetOverrideChooser;

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
  public ArmTargets mArmTargetOverride = ArmTargets.NONE;
  public ArmControlType mArmControlType = ArmControlType.Simple;

  public int mManualTargetOffset = 0;

  public int mManualExtendOffset = 0;

  public boolean mBalanceMode = false;

  public static TuneableNumber mTestArmVoltage = new TuneableNumber("TestArmVoltage",0);

  // Position the Auto Pilot System Wants to Drive to
  public TargetedPositions mTargetedPosition = TargetedPositions.NONE;
  public TargetedObject mCurrentTargetedObject = TargetedObject.CONE;
  public Pose2d mTargetPose = new Pose2d();

  /**
   * On Robot Startup
   */
  @Override
  public void robotInit() {
    // Target Position Chooser - Able to Override the Position
    mTargetedPositionChooser = new SendableChooser<TargetedPositions>();
    mTargetedPositionChooser.setDefaultOption("NONE", TargetedPositions.NONE);
    mTargetedPositionChooser.addOption("NONE", TargetedPositions.NONE);
    mTargetedPositionChooser.addOption("GRID_1", TargetedPositions.GRID_1);
    mTargetedPositionChooser.addOption("GRID_2", TargetedPositions.GRID_2);
    mTargetedPositionChooser.addOption("GRID_3", TargetedPositions.GRID_3);
    mTargetedPositionChooser.addOption("GRID_4", TargetedPositions.GRID_4);
    mTargetedPositionChooser.addOption("GRID_5", TargetedPositions.GRID_5);
    mTargetedPositionChooser.addOption("GRID_6", TargetedPositions.GRID_6);
    mTargetedPositionChooser.addOption("GRID_7", TargetedPositions.GRID_7);
    mTargetedPositionChooser.addOption("GRID_8", TargetedPositions.GRID_8);
    mTargetedPositionChooser.addOption("GRID_9", TargetedPositions.GRID_9);
    mTargetedPositionChooser.addOption("SUBSTATION_LEFT", TargetedPositions.SUBSTATION_LEFT);
    mTargetedPositionChooser.addOption("SUBSTATION_RIGHT", TargetedPositions.SUBSTATION_RIGHT);
    SmartDashboard.putData("Target Position Override", mTargetedPositionChooser);

    // Arm Target Position Chooser - Able to Override the Button
    mArmTargetOverrideChooser = new SendableChooser<ArmTargets>();
    mArmTargetOverrideChooser.setDefaultOption("NONE", ArmTargets.NONE);
    mArmTargetOverrideChooser.addOption("SAFE", ArmTargets.SAFE);
    mArmTargetOverrideChooser.addOption("TOP_SCORING_FRONT", ArmTargets.TOP_SCORING_FRONT);
    mArmTargetOverrideChooser.addOption("MID_SCORING_FRONT", ArmTargets.MID_SCORING_FRONT);
    mArmTargetOverrideChooser.addOption("LOW_SCORING_FRONT", ArmTargets.LOW_SCORING_FRONT);
    mArmTargetOverrideChooser.addOption("TOP_SCORING_BACK", ArmTargets.TOP_SCORING_BACK);
    mArmTargetOverrideChooser.addOption("MID_SCORING_BACK", ArmTargets.MID_SCORING_BACK);
    mArmTargetOverrideChooser.addOption("LOW_SCORING_BACK", ArmTargets.LOW_SCORING_BACK);
    mArmTargetOverrideChooser.addOption("SAFE", ArmTargets.SAFE);
    mArmTargetOverrideChooser.addOption("START", ArmTargets.START);
    mArmTargetOverrideChooser.addOption("INTAKE_FRONT", ArmTargets.INTAKE_FRONT);
    mArmTargetOverrideChooser.addOption("INTAKE_BACK", ArmTargets.INTAKE_BACK);
    mArmTargetOverrideChooser.addOption("INTAKE_GROUND_FRONT", ArmTargets.INTAKE_GROUND_FRONT);
    mArmTargetOverrideChooser.addOption("INTAKE_GROUND_BACK", ArmTargets.INTAKE_GROUND_BACK);
    SmartDashboard.putData("Arm Target Override", mArmTargetOverrideChooser);

    // Start Datalog Manager - removed until wpilib issue is resolved
    //DataLogManager.start();

    // Record both DS control and joystick data
    //DriverStation.startDataLog(DataLogManager.getLog());  
    
    // Real or Simulation Robot
    mRobotState.setRealRobot(mRealRobot);
    mDrive.setRealRobot(mRealRobot);
    mAutoPilot.setRealRobot(mRealRobot);
    mSuperStructure.setRealRobot(mRealRobot);

    // populate autonomous list
    populateAutonomousModes();

    // generate generic auto modes to load into JIT
    TrajectoryGeneratorHelper.generateExampleTrajectories();

    // Configure Arm's Maximum and Minimum Rotation Settings
    mArm.setArmRotationMinimum(Enums.ArmTargets.INTAKE_GROUND_FRONT.armAngle);
    mArm.setArmRotationMaximum(Enums.ArmTargets.START.armAngle);

    // Configure Arm's Maximum and Minimum Extension Settings
    mArm.setArmExtensionMaximum(Constants.Arm.MaxExtensionPosition);
    mArm.setArmExtensionMinimum(Constants.Arm.MinExtensionPosition);

    // reset odometry

    // Reset Drive Sensors

    
    mArm.zeroSensors();
    mDrive.zeroHeading();
    mDrive.zeroEncoders();
  }

  /**
   * Robot Periodic - Called Constantly throughout Robot Operation
   */
  @Override
  public void robotPeriodic() {
    
    // Update Odometry of Robot (only if real)
    if(mRealRobot) mDrive.updateOdometry();

    // Update Robot Field Position
    Pose2d robotPose = mDrive.getPose();

    mField.setRobotPose(robotPose);

    // Set Alliance Color
    mRobotState.setAlliance(DriverStation.getAlliance());

    // Update Robotstate
    mRobotState.update();

    // Update Auto Pilot
    mAutoPilot.setRobotPose(mRobotState.getPose());

    // Update Smartdashboard Overall and Subsystems
    updateSmartdashboard();

    // Get Arm Override (if set)
    mArmTargetOverride = mArmTargetOverrideChooser.getSelected();

    // Currently Targeted Object
    mCurrentTargetedObject = mOperatorInterface.setTargetedObject();

    // Controllable Panel (Turn on Light for Cone)
    mPowerPanel.setSwitchableChannel(mCurrentTargetedObject == TargetedObject.CONE);
    
  }

  private void updateSmartdashboard()
  {
    //SmartDashboard.putData("Field", mField);
    //SmartDashboard.putString("Robot Pose", mField.getRobotPose().toString());
    //SmartDashboard.putNumber("Pigeon Degrees", mPigeon.getYaw().getDegrees());
    //SmartDashboard.putNumber("Pigeon Radians", mPigeon.getYaw().getRadians());
    SmartDashboard.putString("Target Position", mTargetedPosition.toString());
    SmartDashboard.putString("Target Pose", mTargetPose.toString());
    SmartDashboard.putString("Target Arm Position", mCurrentArmTarget.toString());
    SmartDashboard.putNumber("Target Arm Angle", mCurrentArmTarget.armAngle);
    SmartDashboard.putNumber("Target Arm Extension", mCurrentArmTarget.armExtend);
    SmartDashboard.putBoolean("Automatic Mode", mPositionMode);
    SmartDashboard.putNumber("rotate offset", mManualTargetOffset);
    SmartDashboard.putNumber("extend offset", mManualExtendOffset);
    SmartDashboard.putString("Targeted Object", mCurrentTargetedObject.toString());
    SmartDashboard.putBoolean("balance mode", mBalanceMode);
    SmartDashboard.putNumber("pigeon pitch", mPigeon.getUnadjustedPitch().getDegrees());
    
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

    // Superstructure 
    mSuperStructure.updateSmartDashBoard();

    // Iterates each Subsystem 
    mSubsystemManager.updateSmartdashboard();

    // Simulation Only 
    if(!mRealRobot)
    {
      mSimMechanism.updateSmartDashboard();
    }
  }

  // Populate Sendable Chooser of Auto Modes
  private void populateAutonomousModes(){
    // Auto Mode
    mAutoModes = new SendableChooser<AutoModeBase>();
    //mAutoModes.setDefaultOption("Nothing", new DoNothingMode());
    mAutoModes.setDefaultOption("Test Swerve Mode", new SwerveTestAutoMode() );
    SmartDashboard.putData("Auto Modes", mAutoModes);
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

    // Consider alliance Color
    mAutoModeBase.setAllianceColor(mRobotState.getAlliance());

    // Set Starting Pose if Specified
    if(mAutoModeBase.hasStartingPose())
    {
      mRobotState.resetPosition(mAutoModeBase.getStartingPose());
    }

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
    mGrasper.setGrasperFullyClosed();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mOperatorInterface.setDriverRumble(mBalanceMode, mBalanceMode ? .2 : 0);

    // Target Position Chooser
    if (mTargetedPositionChooser.getSelected() == TargetedPositions.NONE) {
      if (mOperatorInterface.getScoringCommand() != TargetedPositions.NONE) {
        mTargetedPosition = mOperatorInterface.getScoringCommand();
      }
    }else {
      // Targeted Position Chooser is not none
      mTargetedPosition = mTargetedPositionChooser.getSelected();
    }

    
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
    
    DriveLoop(mOperatorInterface.getDrivePrecisionSteer(), true);

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
        mArm.setExtensionJog(1);
      } 
      else if (mOperatorInterface.getArmJogRetracted()){
        mArm.setExtensionJog(-1);
      }      
      else {
        mArm.setExtensionJog(0);
      }  


      //Manual rotation of the arm using Y/A
    if (mOperatorInterface.getArmRotateForward()){
      mArm.setShoulderJog(mTestArmVoltage.get());  
    } 
    else if (mOperatorInterface.getArmRotateBackward()) {
      mArm.setShoulderJog(-mTestArmVoltage.get());
    }
    else {
      mArm.setShoulderJog(0);
    }
    /* 
     //Grasper Open/Close using RT
     if (mOperatorInterface.getGrasperModeSwap()){
      mGrasper.setGrasperClosed();

    }
    else if (mOperatorInterface.getGrasperWheelIntake()){
      mGrasper.setGrasperOpen();
    }
    */
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
         mTestArmExtension += 1000;
       }
       else if (mOperatorInterface.setArmExtendedMinusOne()){
         mTestArmExtension -= 1000;
       }
       else if (mOperatorInterface.setArmExtendedPlusFive()){
         mTestArmExtension += 3000;
       }
       else if (mOperatorInterface.setArmExtendedMinusFive()){
         mTestArmExtension -= 3000;
       }      
 
        SmartDashboard.putNumber("testArmPos", mTestTargetPositionDegrees);
        SmartDashboard.putNumber("testExtensionPos", mTestArmExtension);

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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

    // Add April Tag Fields to Camera Sim System
    mPhotonVision.addSimVisionTargets(FieldConstants.aprilTagField);
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
    // This will set the actualll commanded 
    //mSimMechanism.SetArmAngle(mCurrentArmTarget.armAngle + mManualTargetOffset);
    //mSimMechanism.SetArmLength(mCurrentArmTarget.armExtend + mManualExtendOffset);
    mSimMechanism.SetArmAngle(mArm.getArmTargtedDegrees());
    mSimMechanism.SetArmLength(mArm.getArmTargetExtension());

    // Grasper - Open or Close
    if(mGrasper.getGrasperState() == Grasper.GrasperState.Open)
    {
      mSimMechanism.openGrasper();
    } else 
    {
      mSimMechanism.closeGrasper();
    }

    // Process Frame where the Robot currently is
    mPhotonVision.processSimFrame(mField.getRobotPose());
  }

  /** Called Throughout Teleop Periodic */
  private void RobotLoop(){
    // Drive the Robot
    DriveLoop(mOperatorInterface.getDrivePrecisionSteer(), true);

    
    // Get Arm Target
    if(mOperatorInterface.getArmTarget() != ArmTargets.NONE){
      mCurrentArmTarget = mOperatorInterface.getArmTarget();
      mManualExtendOffset = 0;
      mManualTargetOffset = 0;
    }

    // Get Arm Extension Trims
    if(mOperatorInterface.getManualArmExtendUp()){
      mManualExtendOffset += 1;
    }else if(mOperatorInterface.getManualArmExtendDown()){
      mManualExtendOffset -= 1;
    }

    // Get Arm Rotation Trims
    if(mOperatorInterface.getManualArmRotateUp()){
      mManualTargetOffset += 1;
    }else if(mOperatorInterface.getManualArmRotateDown()){
      mManualTargetOffset -= 1;
    }

    // Arm Override by Shuffleboard (if set)
    if(ArmTargets.NONE != mArmTargetOverride) mCurrentArmTarget = mArmTargetOverride;

    // Override for Cube Objects (if exists)
    // Allow Cube Positions to be different
    if(mCurrentTargetedObject == TargetedObject.CUBE)
    {
      // If an override position for cube, get the cube specific target
      if(Constants.Arm.CubeArmTargetOverrides.containsKey(mCurrentArmTarget))
      {
        mCurrentArmTarget = Constants.Arm.CubeArmTargetOverrides.get(mCurrentArmTarget);
      }
    }else {
      // Current Object is Cone, Convert Back to Cube
      if(Constants.Arm.CubeArmTargetOverrides.containsValue(mCurrentArmTarget))
      {
        // Current Arm Target is a Cube, convert back to its cone counterpart
        for (var entry : Constants.Arm.CubeArmTargetOverrides.entrySet()) {
          if(entry.getValue() == mCurrentArmTarget)
          {
            mCurrentArmTarget = entry.getKey();
            break;
          }
        }
      }
    }

    if (mGrasper.getGrasperState() == Grasper.GrasperState.Closed){
      mOperatorInterface.setDriverRumble(true, 0.2);
    }
    // Arm Up After Intake
    if(mCurrentArmTarget == ArmTargets.INTAKE_FRONT && mGrasper.getGrasperState() == GrasperState.FullyClosed){
      mCurrentArmTarget = ArmTargets.POST_INTAKE_FRONT;
    }else if(mCurrentArmTarget == ArmTargets.POST_INTAKE_FRONT && mGrasper.getGrasperState() == GrasperState.Open){
      mCurrentArmTarget = ArmTargets.INTAKE_FRONT;
    }
    if(mCurrentArmTarget == ArmTargets.INTAKE_BACK && mGrasper.getGrasperState() == GrasperState.FullyClosed){
      mCurrentArmTarget = ArmTargets.POST_INTAKE_BACK;
    }else if(mCurrentArmTarget == ArmTargets.POST_INTAKE_BACK && mGrasper.getGrasperState() == GrasperState.Open){
      mCurrentArmTarget = ArmTargets.INTAKE_BACK;
    }

    // Simple Arm Control
    mSuperStructure.setTargetArmPosition(mCurrentArmTarget);
    mSuperStructure.update();

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

    //driver feedback - beam sensor creates vibration (if getSensorBroken == true then vibrate)
    if (mGrasper.getGrasperState() == Grasper.GrasperState.Closed) {
      mOperatorInterface.setDriverRumble(true, 1);
  } else {
    mOperatorInterface.setDriverRumble(false, 0);
  }
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
    SmartDashboard.putNumber("Driver Turn", driveTurn);

    // precision steer (slow down throttle if left trigger is held)
   if(precisionSteer) driveThrottle *= .55;

    boolean wantsAutoSteer = mOperatorInterface.getDriveAutoSteer();
    if(mOperatorInterface.getBalanceMode()){
      mBalanceMode = !mBalanceMode;
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

      // Get Target Pose
      mTargetPose = getTargetPose(mTargetedPosition);

      // Based on Target, Determine the Rotation
      Rotation2d wallRotation = getClosestSideToWall(GameWalls.AllianceWall).getRotation();

      // Create Pose with the corresponding rotation 
      Pose2d calculatedPose = new Pose2d(mTargetPose.getTranslation(), wallRotation);
      
      // Set into AutoPilot
      mAutoPilot.setTargetPose(calculatedPose);

      // Set Final Translation (if applicable)
      mAutoPilot.setFinalTranslation(getFinalTranslation(mTargetedPosition));

      // Update Auto Pilot
      mAutoPilot.update(false);

      // Auto Pilot (and has Valid Target Position)
      if(wantsAutoSteer && mTargetedPosition != TargetedPositions.NONE)
      {
       // Update Auto Pilot (allow drive)
       mAutoPilot.update(true);
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
        mDrive.setBrake(false);
        // Normal Swerve Operation
        
        // Swerve Snap to a Direction (Butttposeon Press Quickly Moves Robot)
        SwerveCardinal snapCardinal = mOperatorInterface.getSwerveSnap();
        if(SwerveCardinal.NONE != snapCardinal) // Snap Direction Detected!
        {
          mDrive.startSnap(snapCardinal.degrees);
        }

        // Swerve Drive
        // Joystick based Translation
        Translation2d sTrans = mOperatorInterface.getSwerveTranslation();
        if(true) sTrans = sTrans.times(.5); // slow down the speed by 50%!
        double sRotation = mOperatorInterface.getSwerveRotation();
        sRotation *= .25;

        // Simple Translation (DPad ... Alternative Control)
        // This allows driver to use DPad to only use in one direction
        // If it is detected that driver is using this, control will defer to this instead
        Translation2d sSimpleTrans = mOperatorInterface.getSimpleSwerveTranslation();
        if(sSimpleTrans.getX() != 0 || sSimpleTrans.getY() != 0)
        {
          // Simple Translation has a nonzero component, use this instead!
          sTrans = sSimpleTrans.times(Constants.SwerveConstants.simpleSwerveDriveSpeed);
        }

        mDrive.setSwerveDrive(sTrans, sRotation, true, true, precisionSteer);

        // Log Inputs
        SmartDashboard.putString("Controls/Swerve Input Translation", sTrans.toString());
        SmartDashboard.putNumber("Controls/Swerve Input Rotation", sRotation);

        // Clear Auto Pilot Sequence Info (since it isn't being used)
        mAutoPilot.clear(); 

        // Reset to current targeted position
        mAutoPilot.reset();
      }
    }
  }

  // Get Target Pose from Target Position
  public Pose2d getTargetPose(TargetedPositions pos)
  {
    Pose2d result = new Pose2d();
    if(TargetedPositions.NONE != pos) 
    {
      // Use Targeted Position to lookup Target Translation
      // These are the translations of the targeted score position
      Translation2d selectedXY = FieldConstants.getTargetPositionPose(pos, DriverStation.getAlliance());
      if(null != selectedXY)
      {
        result = new Pose2d(selectedXY, new Rotation2d());
      }
    }
    return result;
  }

  // Get Final Translation for Position (if it exists)
  public Translation2d getFinalTranslation(TargetedPositions pos)
  {
    Translation2d result = FieldConstants.getTargetPositionFinalTranslation(pos, DriverStation.getAlliance());
    return result;
  }

  // Get Closest Side to Targted Wall
  public SwerveRotation getClosestSideToWall(Enums.GameWalls wall)
  {
    SwerveRotation sRot = SwerveRotation.FRONT_FACING_FORWARD;

    // Get Current Rotation
    Rotation2d curRotation = mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : 
        mPigeon.getSimYaw().getWPIRotation2d();
    double degrees = curRotation.getDegrees();
    double absDegrees = Math.abs(degrees);
    if(absDegrees >= 90)
    {
      sRot = SwerveRotation.FRONT_FACING_GRID;
    }
    return sRot;
  }

}
