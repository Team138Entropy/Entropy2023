package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.Enums.ArmTargets;
import frc.robot.Enums.SwerveCardinal;
import frc.robot.Enums.SwerveRotation;
import frc.robot.Enums.TargetedObject;
import frc.robot.Enums.TargetedPositions;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.ArmAction;
import frc.robot.auto.actions.ChargingStationAction;
import frc.robot.auto.actions.DriveAction;
import frc.robot.auto.actions.DriveToPose;
import frc.robot.auto.actions.DriveTrajectoryAction;
import frc.robot.auto.actions.GrasperAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PickupAction;
import frc.robot.auto.actions.SequentialAction;
import frc.robot.auto.actions.SetPose;
import frc.robot.auto.actions.TargetWaypoint;
import frc.robot.auto.actions.TurnInPlaceAction;
import frc.robot.auto.actions.WaitAction;

public class ChargingStationMode extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();
    
    public ChargingStationMode(TargetedObject gameObject, boolean aquireObject)
    {
        setStartingPosition(TargetedPositions.GRID_5, SwerveRotation.FRONT_FACING_GRID);
        
        // Score Gamepiece
        if(gameObject == TargetedObject.CONE){
            addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT));
        }else{
            addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT_CUBE));
        }
        addAction(new WaitAction(.05));

        // Open the Grasper
        addAction(new GrasperAction(true));

        // Wait for Object to fall
        addAction(new WaitAction(.05));


        addAction(
            new ParallelAction(
                new ArmAction(ArmTargets.HOME_BACKSIDE),
                new SequentialAction(
                    new WaitAction(1.1),
                    new DriveAction(new Translation2d(.5,0),3)
                )
            )
        );

        // Move Arm to Safety
        

        // Drive Over the Ramp (Only Move in the +X)

        // Now would be the Time to attempt an aquire
        addAction(new WaitAction(.05));

       // addAction(new DriveAction(new Translation2d(-.5,0),3));

        // Charging Station Action
        addAction(new ChargingStationAction());




        
    }


    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
}
