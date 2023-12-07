package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Enums.ArmTargets;
import frc.robot.Enums.SwerveRotation;
import frc.robot.Enums.TargetedPositions;
import frc.robot.subsystems.Grasper;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Grasper.GrasperState;
import frc.robot.util.Waypoint;
import frc.robot.vision.AutoPilot;

// Pickup Action
// Drives at Waypoint and Position until a pickup is detected
// Sets Arm into Pickup
public class PickupAction implements Action {

    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Grasper mGrasper = Grasper.getInstance();
    private final ArmTargets mArmTarget;
    private final Waypoint mWaypoint;
    private final TargetWaypoint mTargetWaypointAction;
    private boolean mPickedUp;
    private boolean mAutoPilotFinished;
    

    public PickupAction(Waypoint targetPickupPosition, SwerveRotation rotation, ArmTargets target)
    {
        mWaypoint = targetPickupPosition;
        mArmTarget = target;
        mPickedUp = false;
        mAutoPilotFinished = false;
        mTargetWaypointAction = new TargetWaypoint(targetPickupPosition, rotation);
    }
    
    @Override
    public void start()
    {
        // Tell the Arm to Target Pickup Position
        mSuperstructure.setTargetArmPosition(mArmTarget);
        mSuperstructure.update();

        // Open the Grasper
        mGrasper.setGrasperOpen();

        // Start the Auto Pilot
        mTargetWaypointAction.start();
    }

    @Override 
    public void update() {
        // Continue Updating Arm
        mSuperstructure.update();

        // Continue Targeting Position
        mTargetWaypointAction.update();

        // If the Grasper has been closed, the beam sensor picked it up
        mPickedUp = (mGrasper.getGrasperState() != GrasperState.Open);
    }

    @Override
    public void done() {
    }

    @Override 
    public boolean isFinished() {
        // Arm has reached target angle and extension
        return (mPickedUp || mTargetWaypointAction.isFinished());
    }
}
