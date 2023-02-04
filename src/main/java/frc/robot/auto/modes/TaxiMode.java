package frc.robot.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.DriveTrajectoryAction;

public class TaxiMode extends AutoModeBase {

public TaxiMode(boolean DriveStraight)
    {
        if (DriveStraight == true) {
        
         {Trajectory genTraj = TrajectoryGenerator.generateTrajectory(List.of(

            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(2, 0, new Rotation2d())
        ), Constants.SwerveConstants.AutoConstants.slowSwerveSpeedConfig);

        addAction(new DriveTrajectoryAction(genTraj));
        }
        }
        else {
            {Trajectory genTraj = TrajectoryGenerator.generateTrajectory(List.of(

                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(0, 1, new Rotation2d()),
                new Pose2d(2, 1, new Rotation2d())
            ), Constants.SwerveConstants.AutoConstants.slowSwerveSpeedConfig);
    
            addAction(new DriveTrajectoryAction(genTraj));
            }
        }
    }

    
    
    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
        
    }


    