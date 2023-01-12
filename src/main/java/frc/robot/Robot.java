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
import frc.robot.util.drivers.Pigeon;

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
  private final Arm mArm = Arm.getInstance();

  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;
  private AutoModeBase mAutoModeBase = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  // Acceleratometer 
  private Accelerometer mAccelerometer = new BuiltInAccelerometer();

  // Field Object
  private final Field2d mField = new Field2d();

  private final Pigeon mPigeon = Pigeon.getInstance();

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

    // Reset Drive Sensors
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
    SmartDashboard.putNumber("Drive Throttle", mOperatorInterface.getDriveThrottle());
    SmartDashboard.putNumber("Drive Turn", mOperatorInterface.getDriveTurn());
    SmartDashboard.putData("Field", mField);
    SmartDashboard.putNumber("Pigeon Degrees", mPigeon.getYaw().getDegrees());
    SmartDashboard.putNumber("Pigeon Radians", mPigeon.getYaw().getRadians());
    mArm.updateSmartDashBoard();
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
    if (mOperatorInterface.getArmJogUp()){
      mArm.setArmAngle(45);
    }
      else if (mOperatorInterface.getArmJogDown()){
      mArm.setArmAngle(135); 
    }
      else {mArm.setArmAngle(90);
      }

  }

  
  


  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

    // Update Pose on Virtual Field
    
  }

  /** Called Throughout Teleop Periodic */
  private void RobotLoop(){

    // Drive the Robot

    // Update Robot Field Position
  }

   /**
   * DriveLoop
   * precisionSteer - Tunes Down Throttle. Useful for precise movements
   * allowAutoSteer - Enables/Disables AutoSteering
   */
  
}
