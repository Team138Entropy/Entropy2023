package frc.robot.auto;


import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import  edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveStyle;
import frc.robot.util.DriveSignal;

import java.util.List;

// Singelton Instance of a Trajectory Follower
// This drives the drivebase
public class TrajectoryFollower {
    private static TrajectoryFollower mInstance;

    // Trajectory to Drive
    private Trajectory mTrajectory;

    // The Ramsete Controller to follow the trajectory.
    // RamsetController for Differential/West Coast Drive Trains
    private final RamseteController mRamseteController;

    // Swerve 
    // HolonomicDriveController for Swerve/Mechanum based systems
    private HolonomicDriveController mHolonomicDriveController;
    private ProfiledPIDController mSwerveThetaController;
    private PIDController mSwerveXPidController;
    private PIDController mSwerveYPidController;


    // The timer to use during the autonomous period.
    private Timer mTimer;

    // Create Field2d for robot and trajectory visualizations.
    private Field2d mField;

    // Reference to the Drive Subsystem
    private final Drive mDrive = Drive.getInstance();

    // Trajectory Complete
    private boolean mComplete;

    // Allow Trajectory Follower to Run
    private boolean mRun;

    public static synchronized TrajectoryFollower getInstance() {
       if (mInstance == null) {
         mInstance = new TrajectoryFollower();
       }
       return mInstance;
    }

    private TrajectoryFollower(){
        // Create the Ramsete Controller
        mRamseteController = new RamseteController();

        // initialize trajectory follower
        init();
    }

    // Sets Selected Trajectory
    public void setTrajectory(Trajectory traj){
        mTrajectory = traj;

        // Push the trajectory to Field2d.
        mField.getObject("traj").setTrajectory(mTrajectory);
    }
    
    private void init(){
        System.out.println("TrajectorFollower::Init");
        // Create and push Field2d to SmartDashboard.
        mField = new Field2d();
        SmartDashboard.putData("Autonomous Field", mField);

        // Swerve Specific
        // Rotation Controller
        mSwerveThetaController = new ProfiledPIDController(
                Constants.SwerveConstants.AutoConstants.kPThetaController, 0, 0,
                Constants.SwerveConstants.AutoConstants.kThetaControllerConstraints);
        mSwerveThetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Swerve Translation X and Y PID Controllers
        mSwerveXPidController = new PIDController(Constants.SwerveConstants.AutoConstants.kPXController, 0, 0);
        mSwerveYPidController = new PIDController(Constants.SwerveConstants.AutoConstants.kPYController, 0, 0);

        // Swerve Specific Controller
        mHolonomicDriveController = new HolonomicDriveController(mSwerveXPidController, 
                                        mSwerveYPidController, mSwerveThetaController);
    }

    // Called Once at the Start of following the Path
    public void Start(){
        System.out.println("TrajectoryFollower::Start");
        // Mark Trajectory as Incomplete
        mComplete = false;

        // Allow Trajectory to run
        mRun = true;

        // Initialize the timer.
        mTimer = new Timer();
        mTimer.start();
        
        // TODO - no for swerve?
        // Reset Encoder Values
        //mDrive.zeroEncoders();

        // Zero Gyro Position
        //mDrive.zeroHeading();

        // Reset the drivetrain's odometry to the starting pose of the trajectory.
        //mDrive.resetOdometry(mTrajectory.getInitialPose());

        // Store the Current Pose of the Drive
        mDrive.storeCurrentPose();

        // Update the Smartdashboard
        updateSmartdashboard();
    }

    public void Update(){
        System.out.println("TrajectoryFollower::Update");
        // Update the Drives Odometry
        mDrive.updateOdometry();

        // Update the Robot Position on Field2D
        mField.setRobotPose(mDrive.getPose());
        System.out.println("robot pose done");

        // if the time is within the total trajectory time
        if (mRun && mTimer.get() < mTrajectory.getTotalTimeSeconds()) {
            System.out.println("Timer Seconds: " + mTimer.get());
            System.out.println("Total Seconds: " + mTrajectory.getTotalTimeSeconds());

            // Get the desired pose from the trajectory.
            var currentDrivePose = mDrive.getPose();
            var desiredPose = mTrajectory.sample(mTimer.get());
            Rotation2d desiredRotation = new Rotation2d();


            // Drive System Specific Logic
            if(DriveStyle.DIFFERENTIAL_DRIVE == mDrive.getDriveStyle())
            {
                // Get the reference chassis speeds from the Ramsete controller.
                var refChassisSpeeds = mRamseteController.calculate(mDrive.getPose(), desiredPose);
                
                // Set the linear and angular speeds.
                mDrive.automousDrive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
            }
            else if(DriveStyle.SWERVE_DRIVE == mDrive.getDriveStyle())
            {
                // Calculate Swerve Chasis Speeds
                ChassisSpeeds calculatedSpeeds = mHolonomicDriveController.calculate(currentDrivePose, desiredPose, desiredRotation);
                var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

                // Set Swerve to those Module States
                mDrive.setModuleStates(targetSwerveModuleStates);
            }
          } else {
            // Log the Trajectory Follower Completeing the Path
            System.out.println("TrajectoryFollower::PathComplete");
            
            // mark path as complete
            mComplete = true;

            // Set Drive System to do Nothing
            // Drive System Specific Logic
            if(DriveStyle.DIFFERENTIAL_DRIVE == mDrive.getDriveStyle())
            {
                mDrive.automousDrive(0, 0);
            }
            else if(DriveStyle.SWERVE_DRIVE == mDrive.getDriveStyle())
            {
                mDrive.setSwerveDrive(new Translation2d(), 0, true, true);
            }
        }

        // Update the Smartdashboard
        updateSmartdashboard();
    }

    // Stop Drivetrain from moving
    public void StopDrive(){
        System.out.println("TrajectoryFollower::StopDrive");
        mRun = false;
        mDrive.setDrive(0, 0, false);
    }

    //returns if getComplete is done
    public boolean isComplete(){
        return mComplete;
    }

    public void updateSmartdashboard()
    {
        final String key = "TrajectorFollower/";
        SmartDashboard.putBoolean(key + "Complete", mComplete);
        SmartDashboard.putBoolean(key + "Running", mRun);
    }


}
