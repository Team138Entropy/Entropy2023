package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
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
    private Timer mTimer = new Timer();


    public GrasperAction(boolean open)
    {
       mOpen = open;
    }
    
    @Override
    public void start()
    {
      mTimer.reset();
      mTimer.start();
      
       if(mOpen){ 
        // Open the Grasper
        mGrasper.setGrasperOpen();
       }else{ 
        // Close the Grasper
        mGrasper.setGrasperClosed();
       }
    }

    @Override 
    public void update() {
      //mGrasper.setGrasperWheelIntake();
    }

    @Override
    public void done() {
      //mGrasper.cancelGrasperWheelIntake();
      
    }

    @Override 
    public boolean isFinished() {
      return mTimer.hasElapsed(.1);
    }
}
