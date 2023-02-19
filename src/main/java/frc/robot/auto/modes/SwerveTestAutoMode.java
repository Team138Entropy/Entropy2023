package frc.robot.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Enums.SwerveRotation;
import frc.robot.Enums.TargetedPositions;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.DriveToPose;
import frc.robot.auto.actions.DriveTrajectoryAction;
import frc.robot.auto.actions.SetPose;
import frc.robot.auto.actions.TargetWaypoint;
import frc.robot.auto.actions.WaitAction;

public class SwerveTestAutoMode extends AutoModeBase {
    
    public SwerveTestAutoMode()
    {
        // this Auto Mode will Start in Front of Grid 5 with the front facing it
        setStartingPosition(TargetedPositions.GRID_5, SwerveRotation.FRONT_FACING_GRID);

        // Rotate to the High Scoring Position

        // Wait to make sure we are good!
        addAction(new WaitAction(5));

        // Eject!

        // Wait a small delay to make sure we scored
        
        // Drive to the Pickup Section
        addAction(new TargetWaypoint(FieldConstants.Auto.Waypoints.CommunityExitLeft));

        /*
        addAction(new SetPose(new Pose2d(
            new Translation2d(3, 5),
            new Rotation2d()        
        )));
        addAction(new WaitAction(10));

        addAction(new DriveToPose(new Pose2d(
            new Translation2d(5,5),
            new Rotation2d()        
        )));

        addAction(new WaitAction(1));

        addAction(new DriveToPose(new Pose2d(
            new Translation2d(5,6),
            new Rotation2d()        
        )));
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
