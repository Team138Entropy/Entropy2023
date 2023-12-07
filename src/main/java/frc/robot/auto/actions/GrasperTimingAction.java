package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Enums.ArmTargets;
import frc.robot.Enums.TargetedPositions;
import frc.robot.subsystems.Grasper;
import frc.robot.subsystems.Superstructure;
import frc.robot.vision.AutoPilot;

// Sets the Grapser Timing Delay
public class GrasperTimingAction implements Action {
    private final Grasper mGrasper = Grasper.getInstance();
    private final double mDelaySeconds;

    public GrasperTimingAction(double delay)
    {
        mDelaySeconds = delay;
    }
    
    @Override
    public void start()
    {
        mGrasper.mDelaySeconds = mDelaySeconds;
    }

    @Override 
    public void update() {   }

    @Override
    public void done() {    }

    @Override 
    public boolean isFinished() {
      return true;
    }
}
