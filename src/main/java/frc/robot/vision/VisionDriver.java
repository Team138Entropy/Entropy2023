package frc.robot.vision;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.auto.TrajectoryFollower;

// Vision Driver System
// Need some sort of force regenerate based on a path reupdate
public class VisionDriver {
    private static VisionDriver mInstance;

    public static synchronized VisionDriver getInstance() {
        if (mInstance == null) {
          mInstance = new VisionDriver();
        }
        return mInstance;
    }

    private final TrajectoryFollower mTrajectoryFollower = TrajectoryFollower.getInstance();
    private boolean mRunning;
    private Trajectory mTrajectory;
    private boolean mTrajectorySet;
    private Pose2d mStartingPose;
    private Pose2d mTargetedPose;

    // Motion Control
    private static final TrapezoidProfile.Constraints mX_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints mY_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints mOMEGA_CONSTRAINTS =   
            new TrapezoidProfile.Constraints(8, 8);
    private final ProfiledPIDController mXController = new ProfiledPIDController(3, 0, 0, mX_CONSTRAINTS);
    private final ProfiledPIDController mYController = new ProfiledPIDController(3, 0, 0, mY_CONSTRAINTS);
    private final ProfiledPIDController mOmegaController = new ProfiledPIDController(2, 0, 0, mOMEGA_CONSTRAINTS);

    // Type of System being used to Drive
    enum VisionDriveMode {
        TrajectoryFollower,
        MotionControl
    };
    private VisionDriveMode mDriveMode;

    private VisionDriver()
    {
        init();
    }

    private void init()
    {
        // Not Currently Running
        mRunning = false;
        mTrajectorySet = false;

        // Drive Style
        mDriveMode = VisionDriveMode.TrajectoryFollower;

        // Placeholder Pose Targets
        mTargetedPose = new Pose2d();
        mStartingPose = new Pose2d();

        // Motion Control Init
        mXController.setTolerance(0.2);
        mYController.setTolerance(0.2);
        mOmegaController.setTolerance(Units.degreesToRadians(3));
        mOmegaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTargetPose(Pose2d targPose)
    {
        mTargetedPose = targPose;
    }

    public void setStartingPose(Pose2d startPose)
    {
        mStartingPose = startPose;
    }

    private void start()
    {
        // Not Currently Running!
        mRunning = true;

        // Proceed based on Type
        if(VisionDriveMode.TrajectoryFollower == mDriveMode)
        {
            // Generate a Trajectory to Drive
            mTrajectory = TrajectoryGenerator.generateTrajectory(
                    List.of(mStartingPose, mTargetedPose),
                    Constants.SwerveConstants.AutoConstants.slowSwerveSpeedConfig
            );
            mTrajectorySet = true;

            // Set Trajectory to Follow
            mTrajectoryFollower.setTrajectory(mTrajectory);

            // Start Trajectory Folling
            mTrajectoryFollower.Start();
        } else if(VisionDriveMode.MotionControl == mDriveMode)
        {

        }
    }

    public void update() 
    {
        // Start if not started
        if(!mRunning)
        {
            start();
        }

        // Keep Following the Trajectory
        mTrajectoryFollower.Update();
    }

    public void stop()
    {
        mRunning = false;
        mTrajectorySet = false;

        // Stop the Trajectory Follower
        mTrajectoryFollower.Stop();
    }

    public boolean getRunning()
    {
        return mRunning;
    }

    public Trajectory getTrajectory()
    {
        return mTrajectorySet ? mTrajectory : null;
    }

    public void updateSmartDashBoard()
    {
        final String key = "VisionDriver/";
        SmartDashboard.putBoolean(key + "Running", mRunning);
        SmartDashboard.putString(key + "Mode", mDriveMode.toString());
        SmartDashboard.putBoolean(key + "Has Trajectory", mTrajectorySet);
        SmartDashboard.putString(key + "Trajectory Starting Pose", 
            mTrajectorySet ? mTrajectory.getInitialPose().toString() : ""
        );
        SmartDashboard.putString(key + "Target Pose", mTargetedPose.toString());
        SmartDashboard.putString(key + "Starting Pose", mStartingPose.toString());
    }
}
