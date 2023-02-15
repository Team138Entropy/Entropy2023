package frc.robot.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.DriveToPose;
import frc.robot.auto.actions.DriveTrajectoryAction;
import frc.robot.auto.actions.SetPose;
import frc.robot.auto.actions.WaitAction;

public class SwerveTestAutoMode extends AutoModeBase {
    
    public SwerveTestAutoMode()
    {

        /*
        // Make a Slow Swerve Trajectory that wants to go up field 3 meters?
        Trajectory genTraj = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(1.5, 1, new Rotation2d()),
            new Pose2d(1.5, 0, new Rotation2d())
        ), Constants.SwerveConstants.AutoConstants.slowSwerveSpeedConfig);

        addAction(new DriveTrajectoryAction(genTraj));
        */

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



    }


    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
}
