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

public class SwerveTestAutoMode extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();
    
    public SwerveTestAutoMode()
    {
        // this Auto Mode will Start in Front of Grid 5 with the front facing it
        setStartingPosition(TargetedPositions.GRID_5, SwerveRotation.FRONT_FACING_GRID);
        /* 
        addAction(new ArmAction(ArmTargets.MID_SCORING_FRONT));

        // Wait to make sure we are good!
        addAction(new WaitAction(.2));

        // Open the Grasper
        addAction(new GrasperAction(true));

        // Wait for Object to fall
        addAction(new WaitAction(.2));

        addAction(new ArmAction(ArmTargets.HOME_BACKSIDE));  
        */

        Translation2d scoreSpot = FieldConstants.getTargetPositionPose(TargetedPositions.GRID_5, mRobotState.getAlliance());

        

        
        DriveTrajectoryAction trajAction = new DriveTrajectoryAction(SwerveRotation.FRONT_FACING_GRID.getRotation(),.8,2);
        trajAction.addPose(
            new Pose2d(
                scoreSpot,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        trajAction.addPose(
            new Pose2d(
                scoreSpot.plus(new Translation2d(.5,0)),
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );

        trajAction.generate();
        addAction(trajAction);
        addAction(new TurnInPlaceAction(SwerveCardinal.FORWARDS));

        addAction(new DriveAction(new Translation2d(.4,0),3.3));

        addAction(new ChargingStationAction());

        


        // Score High
        /*
        addAction(new ArmAction(ArmTargets.MID_SCORING_FRONT));

        // Wait to make sure we are good!
        addAction(new WaitAction(.2));

        // Open the Grasper
        addAction(new GrasperAction(true));

        // Wait for Object to fall
        addAction(new WaitAction(.2));

        addAction(new ArmAction(ArmTargets.HOME_BACKSIDE));
        */


        /* 
        // Drive to Community Entrance and Put Arm in Rear Pickup
        ArrayList<Action> ParallelActions1 = new ArrayList<>();
        ParallelActions1.add(
            new TargetWaypoint(FieldConstants.Auto.Waypoints.CommunityEntranceRight,
                SwerveRotation.FRONT_FACING_GRID
            )
        );
        ParallelActions1.add(
            new ArmAction(ArmTargets.INTAKE_GROUND_BACK)
        );
        addAction(new ParallelAction(ParallelActions1));
        
        // Pickup Cube (Furthest to the Right)
        addAction(new PickupAction(FieldConstants.Auto.Waypoints.StagingWaypoints[0], 
          SwerveRotation.FRONT_FACING_GRID, ArmTargets.INTAKE_GROUND_BACK));

        addAction(
            new TargetWaypoint(FieldConstants.Auto.Waypoints.CommunityEntranceRight,
            SwerveRotation.FRONT_FACING_GRID
        )
        );


        // Drive to Score it!
        /*
        addAction(new TargetWaypoint(FieldConstants.Auto.Waypoints.Gr,
            SwerveRotation.FRONT_FACING_GRID
        ));

        */
        /* 
        DriveTrajectoryAction trajAction = new DriveTrajectoryAction(SwerveRotation.FRONT_FACING_GRID.getRotation());
        trajAction.addPose(getStartingPose());        
        trajAction.addWaypoint(FieldConstants.Auto.Waypoints.CommunityExitRight, SwerveRotation.FRONT_FACING_GRID);
        trajAction.addWaypoint(FieldConstants.Auto.Waypoints.CommunityEntranceRight, SwerveRotation.FRONT_FACING_GRID);
        trajAction.generate();
        addAction(trajAction);
        */

       


    }


    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
}
