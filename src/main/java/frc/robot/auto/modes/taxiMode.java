package frc.robot.auto.modes;


import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.DriveAction;

public class taxiMode extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();
    
    public taxiMode(){
        addAction(new DriveAction(new Translation2d(-.75,0),3.3));
    }


    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
}
