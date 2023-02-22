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


public class StartG9_ScoreH9_IntakeO4_Charge extends AutoModeBase {

    public StartG9_ScoreH9_IntakeO4_Charge(){

        setStartingPosition(TargetedPositions.GRID_9, SwerveRotation.FRONT_FACING_GRID);

        //addAction(new ScoreAction(TargetedObject.CONE ,ArmTargets.TOP_SCORING_HIGH));

        //addAction(new DriveToPose(drive to where object 4 is and facing forward));

        //addAction(new Intake(ArmTargets.GROUND_INTAKE_FRONT));

        //addAction(new MoveArmAction(ArmTargets.HOME_BACKSIDE))

        //addAction(new DriveToPose(Charging station, back facing charging station));

        //addAction(new GamerModeAction)

        

    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }

    
}
