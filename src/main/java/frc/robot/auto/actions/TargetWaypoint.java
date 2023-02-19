package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.Enums.SwerveRotation;
import frc.robot.util.Waypoint;
import frc.robot.vision.AutoPilot;

public class TargetWaypoint implements Action {
    // References
    private final RobotState mRobotState = RobotState.getInstance();
    private final AutoPilot mAutoPilot = AutoPilot.getInstance();

    private final Waypoint mTargetWaypoint;
    private Pose2d mTargetPose;

    public TargetWaypoint(Waypoint wp)
    {
        mTargetWaypoint = wp;
    }

    @Override
    public void start()
    {
        // Get Pose to Target based on Alliance Color
        //  Because the field is flipped this must be accounted for
        mTargetPose = mTargetWaypoint.getPose(mRobotState.getAlliance());

        // Set Target Pose
        mAutoPilot.setTargetPose(mTargetPose);
    }

    @Override 
    public void update() {
        mAutoPilot.update();
    }

    @Override
    public void done() {
        
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

   


}