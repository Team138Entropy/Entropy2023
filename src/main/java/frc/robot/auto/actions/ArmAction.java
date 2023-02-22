package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Enums.ArmTargets;
import frc.robot.Enums.TargetedPositions;
import frc.robot.subsystems.Superstructure;
import frc.robot.vision.AutoPilot;

// Arm Action 
// Moves the Arm
public class ArmAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final ArmTargets mTargetArmPosition;

    public ArmAction(ArmTargets armTarget)
    {
        mTargetArmPosition = armTarget;
    }
    
    @Override
    public void start()
    {
        mSuperstructure.setTargetArmPosition(mTargetArmPosition);
        mSuperstructure.update();
    }

    @Override 
    public void update() {
        mSuperstructure.update();
    }

    @Override
    public void done() {
    }

    @Override 
    public boolean isFinished() {
        // Arm has reached target angle and extension
        return mSuperstructure.isAtTarget();
    }
}
