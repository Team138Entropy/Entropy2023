package frc.robot.vision;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryFollower;

// Vision Driver System
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

    private VisionDriver()
    {
        init();
    }

    private void init()
    {
        // Not Currently Running
        mRunning = false;
        mTrajectorySet = false;

        // Placeholder Pose Targets
        mTargetedPose = new Pose2d();
        mStartingPose = new Pose2d();
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
    }

    public boolean getRunning()
    {
        return mRunning;
    }

    public void updateSmartDashBoard()
    {
        final String key = "VisionDriver/";
        SmartDashboard.putBoolean(key + "Running", mRunning);
        SmartDashboard.putBoolean(key + "Has Trajectory", mTrajectorySet);
        SmartDashboard.putString(key + "Trajectory Starting Pose", 
            mTrajectorySet ? mTrajectory.getInitialPose().toString() : ""
        );
        SmartDashboard.putString(key + "Target Pose", mTargetedPose.toString());
        SmartDashboard.putString(key + "Starting Pose", mStartingPose.toString());
    }
}
