package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.AutoPilot;

// Drives to a Pose
// Optional Argument to Match Original Pose? (avoid the robot from spinning)
public class DriveToPose implements Action {
    private AutoPilot mAutoPilot = AutoPilot.getInstance();

    private Pose2d mTargetPose;
    private Boolean mHasMaxSpeed = false;

    public DriveToPose(Pose2d targetPose)
    {
        mTargetPose = targetPose;
    }
    
    @Override
    public void start()
    {
        // Where are we going!
        mAutoPilot.setTargetPose(mTargetPose);
    }

    @Override 
    public void update() {
        mAutoPilot.update(true);
    }

    @Override
    public void done() {
        
    }

    @Override 
    public boolean isFinished() {
        return mAutoPilot.atTarget();
    }
}
