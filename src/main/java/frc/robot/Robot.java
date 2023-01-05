package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Enums.SwerveCardinal;
import frc.robot.OI.OperatorInterface;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.TrajectoryGeneratorHelper;
import frc.robot.auto.modes.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive.DriveStyle;

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

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  
  // Subsystems
  private final Drive mDrive = Drive.getInstance();

  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;
  private AutoModeBase mAutoModeBase = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  // Acceleratometer 
  private Accelerometer mAccelerometer = new BuiltInAccelerometer();

  // Field Object
  private final Field2d mField = new Field2d();

  // Real Robot Indicator
  private final boolean mRealRobot = Robot.isReal();

  // Various Variables
  int mRumbleTimer = 0;


  /**
   * On Robot Startup
   */
  @Override
  public void robotInit() {
    // Start Datalog Manager
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());    

    // populate autonomous list
    populateAutonomousModes();

    // generate generic auto modes to load into JIT
    TrajectoryGeneratorHelper.generateExampleTrajectories();

    // reset odometry
    mDrive.resetOdometry(new Pose2d());

    // Reset Drive Sensors
    mDrive.zeroSensors();
  }


  /**
   * Robot Periodic - Called Constantly throughout Robot Operation
   */
  @Override
  public void robotPeriodic() {

    // Update Smartdashboard Overall and Subsystems
    updateSmartdashboard();
  }

  private void updateSmartdashboard()
  {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putData("power panel",Constants.Grasper.globelPowerDistribution);
    SmartDashboard.putNumber("accel X", mAccelerometer.getX());
    SmartDashboard.putNumber("accel Y", mAccelerometer.getY());
    SmartDashboard.putNumber("accel Z", mAccelerometer.getZ());
    //SmartDashboard.putBoolean("isTipping", robotTippingCheck());
    SmartDashboard.putNumber("drive throttle", mOperatorInterface.getDriveThrottle());
    SmartDashboard.putNumber("drive turn", mOperatorInterface.getDriveTurn());
    SmartDashboard.putData("Field", mField);
    
    // Iterates each Subsytem 
    mSubsystemManager.updateSmartdashboard();
  }

  // Fill Autonomous Modes List
  private void populateAutonomousModes(){
    // Auto Mode
    mAutoModes = new SendableChooser<AutoModeBase>();
    mAutoModes.setDefaultOption("Nothing", new DoNothingMode());
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
    // dsable operator rumble    
    mOperatorInterface.setOperatorRumble(false);
        
    // zero sensors (if not zero'ed prior on this powerup)
    mSubsystemManager.zeroSensorsIfFresh();
    
    // Disable Auto Thread (if running)
    if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
    }

    // Zero Drive Sensors
    mDrive.zeroSensors();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
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

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    mDrive.updateDriveSim();

    // Update Pose on Virtual Field
    
  }

  /** Called Throughout Teleop Periodic */
  private void RobotLoop(){

    // Drive the Robot
    DriveLoop(mOperatorInterface.getDrivePrecisionSteer(), false);

    // Update Robot Field Position
    mField.setRobotPose(mDrive.getPose());
  }

   /**
   * DriveLoop
   * precisionSteer - Tunes Down Throttle. Useful for precise movements
   * allowAutoSteer - Enables/Disables AutoSteering
   */
  private void DriveLoop(boolean precisionSteer, boolean allowAutoSteer){
    double driveThrottle = mOperatorInterface.getDriveThrottle()*-1;
    double driveTurn = mOperatorInterface.getDriveTurn();

    // precision steer (slow down throttle if left trigger is held)
   if(precisionSteer) driveThrottle *= .3;

    boolean wantsAutoSteer = mOperatorInterface.getDriveAutoSteer();
    wantsAutoSteer &= allowAutoSteer; //disable if autosteer isn't allowed
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
    
    /*
    if(wantsAutoSteer && validTargetInfo){        
      if(ti.isValid()){ //only allow if valud packet
        // autonomously steering robot towards cargo
        // todo: only allow drive in a certain direction? 
       //mDrive.autoSteer(driveThrottle * .4, ti.getErrorAngle());
       mDrive.driveErrorAngle(driveThrottle * .4, ti.getErrorAngle());
      }else{
        System.out.println("Invalid Packet!");
      }
    }else if(wantsAutoSteer){
      // wants auto steer, but invalid target info
      // TODO: vibrate controller so driver knows
    }else{
      //manual drive
      mDrive.setDrive(driveThrottle, driveTurn, false);
    }
    */

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
      mDrive.setBrake(mOperatorInterface.getBrake());

      // Swerve Snap to a Direction (Button Press Quickly Moves Robot)
      SwerveCardinal snapCardinal = mOperatorInterface.getSwerveSnap();
      if(SwerveCardinal.NONE != snapCardinal) // Snap Direction Detected!
      {
        mDrive.startSnap(snapCardinal.degrees);
      }

      // Swerve Drive
      Translation2d sTrans = mOperatorInterface.getSwerveTranslation();
      double sRotation = mOperatorInterface.getSwerveRotation();
      mDrive.setSwerveDrive(sTrans, sRotation, true, true);

      // Log Inputs
      SmartDashboard.putString("Swerve Input Translation", sTrans.toString());
      SmartDashboard.putNumber("Swerve Input Rotation", sRotation);
    }

    // Update Odometry of Robot (only if real)
    if(mRealRobot) mDrive.updateOdometry();
  }

}
