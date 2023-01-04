package frc.robot.auto;


import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import  edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drive;

import java.util.List;

// Singelton Instance of a Trajectory Follower
// This drives the drivebase
public class TrajectoryFollower {
    private static TrajectoryFollower mInstance;

    // Trajectory to Drive
    private Trajectory mTrajectory;

    // The Ramsete Controller to follow the trajectory.
    private final RamseteController mRamseteController;

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

        // Reset Encoder Values
        mDrive.zeroEncoders();

        // Zero Gyro Position
        mDrive.zeroHeading();

        // Reset the drivetrain's odometry to the starting pose of the trajectory.
        mDrive.resetOdometry(mTrajectory.getInitialPose());

        // Store the Current Pose of the Drive
        mDrive.storeCurrentPose();
    }

    public void Update(){
        System.out.println("TrajectoryFollower::Update");
        // Update the Drives Odometry
        mDrive.updateOdometry();
        System.out.println("set robot pose");
        // Update the Robot Position on Field2D
        mField.setRobotPose(mDrive.getPose());
        System.out.println("robot pose done");
        // if the time is within the total trajectory time
        if (mRun && mTimer.get() < mTrajectory.getTotalTimeSeconds()) {
            System.out.println("Timer Seconds: " + mTimer.get());
            System.out.println("Total Seconds: " + mTrajectory.getTotalTimeSeconds());

            // Get the desired pose from the trajectory.
            var desiredPose = mTrajectory.sample(mTimer.get());
                  
            // Get the reference chassis speeds from the Ramsete controller.
            var refChassisSpeeds = mRamseteController.calculate(mDrive.getPose(), desiredPose);
            
            // Set the linear and angular speeds.
            mDrive.automousDrive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
          } else {
            // Log the Trajectory Follower Completeing the Path
            System.out.println("TrajectoryFollower::PathComplete");
            
            // mark path as complete
            mComplete = true;

            // set drive to do nothing
            mDrive.automousDrive(0, 0);
          }
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


}
