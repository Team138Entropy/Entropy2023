package frc.robot.auto.modes;

import java.util.List;

import frc.robot.subsystems.Arm;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Enums.ArmTargets;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.ArmAction;
import frc.robot.auto.actions.DriveTrajectoryAction;
import frc.robot.auto.actions.GrasperEjectAction;
import frc.robot.auto.actions.GrasperIntakeAction;

public class ScoreOneObjectMode extends AutoModeBase {


    public ScoreOneObjectMode(boolean BottomField) {
      if (BottomField == true) {
        addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT));
        addAction(new GrasperEjectAction());
          {Trajectory genTraj = TrajectoryGenerator.generateTrajectory(List.of(

           new Pose2d(0, 0, new Rotation2d()), //starts at bottom scoring
           new Pose2d(2, 0, new Rotation2d())  //needs to go to object start
        ), Constants.SwerveConstants.AutoConstants.slowSwerveSpeedConfig);        
         addAction(new DriveTrajectoryAction(genTraj)); }
        addAction(new ArmAction(ArmTargets.INTAKE_GROUND_BACK));
        addAction(new GrasperIntakeAction());
          {Trajectory genTraj = TrajectoryGenerator.generateTrajectory(List.of(

           new Pose2d(2, 0, new Rotation2d()), //starts at object start
           new Pose2d(0, 0, new Rotation2d())  //needs to go to bottom scoring
        ), Constants.SwerveConstants.AutoConstants.slowSwerveSpeedConfig); 
        addAction(new ArmAction(ArmTargets.MID_SCORING_FRONT));
        addAction(new GrasperEjectAction());
      

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