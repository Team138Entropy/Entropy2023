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
import frc.robot.auto.actions.SetPose;
import frc.robot.auto.actions.TargetWaypoint;
import frc.robot.auto.actions.TurnInPlaceAction;
import frc.robot.auto.actions.WaitAction;

public class ScoreHighAndTaxi extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();
    
    public ScoreHighAndTaxi(TargetedObject gameObject)
    {
        setStartingPosition(TargetedPositions.GRID_3, SwerveRotation.FRONT_FACING_GRID);
        addAction(new GrasperAction(false));
        if(gameObject == TargetedObject.CONE){
            addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT));
        }else{
            addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT_CUBE));
        }
        
        // Wait to make sure we are good!
        addAction(new WaitAction(.2));

        // Open the Grasper
        addAction(new GrasperAction(true));

        // Wait for Object to fall
        addAction(new WaitAction(.2));

        // Close Grasper
        addAction(new GrasperAction(false));

        // Move Arm to backside
        addAction(new ArmAction(ArmTargets.HOME_BACKSIDE));  

        //TODO: joe is this ok or is there a better way to do it?

        addAction(new DriveAction(new Translation2d(.75,0),3.3));

        addAction(new TurnInPlaceAction(SwerveCardinal.RIGHT));
        addAction(new ArmAction(ArmTargets.INTAKE_GROUND_FRONT_CUBE));

        
    }


    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
}
