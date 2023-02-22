package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Enums.ArmTargets;
import frc.robot.Enums.TargetedPositions;
import frc.robot.subsystems.Grasper;
import frc.robot.subsystems.Superstructure;
import frc.robot.vision.AutoPilot;

// Grasper Action
// Opens and Closes the Grasper
public class GrasperAction implements Action {
    private final Grasper mGrasper = Grasper.getInstance();
    private final boolean mOpen;

    public GrasperAction(boolean value)
    {
       mOpen = value;
    }
    
    @Override
    public void start()
    {
       if(mOpen)
       { // Open the Grasper
         mGrasper.setGrasperOpen();
       } else 
       { 
        // Close the Grasper
        mGrasper.setGrasperClosed();
       }
    }

    @Override 
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override 
    public boolean isFinished() {
        return true;
    }
}
