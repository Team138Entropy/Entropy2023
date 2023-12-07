package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    // Target Waypoint
    public TargetWaypoint(Waypoint wp)
    {
        mTargetWaypoint = wp;
    }

    // Target Waypoint, Swwerve Rotations
    public TargetWaypoint(Waypoint wp, SwerveRotation blueRotation, SwerveRotation redRotation)
    {
        mTargetWaypoint = wp;
        mTargetWaypoint.setRotation(blueRotation.getRotation(), Alliance.Blue);
        mTargetWaypoint.setRotation(redRotation.getRotation(), Alliance.Red);
    }

    public TargetWaypoint(Waypoint wp, SwerveRotation rotation) 
    {
        this(wp, rotation, rotation);
    }

    @Override
    public void start()
    {
        // Get Pose to Target based on Alliance Color
        //  Because the field is flipped this must be accounted for
        mTargetPose = mTargetWaypoint.getPose(mRobotState.getAlliance());

        // Force Tolerances to Clear
        mAutoPilot.resetTolerances();

        // Reset Goal
        mAutoPilot.reset();

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
        return mAutoPilot.atTarget();
    }

   


}