package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Enums;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.Enums.ArmTargets;
import frc.robot.Enums.GamePiece;
import frc.robot.Enums.SwerveRotation;
import frc.robot.Enums.TargetedPositions;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.ArmAction;
import frc.robot.auto.actions.DriveTrajectoryAction;
import frc.robot.auto.actions.GrasperAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SequentialAction;
import frc.robot.auto.actions.WaitAction;

public class CableCover2High extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();

    // Cable Cover 2 High
    public CableCover2High( )
    {
        // Set Odometry Starting Location
        setStartingPosition(
            TargetedPositions.GRID_9, 
            SwerveRotation.FRONT_FACING_GRID
        );

        // Current Alliance of this Auto Mode
        Alliance currentAlliance = mRobotState.getAlliance();

        // Charging Station 'Safe' Points
        // Don't get too close to the charging station!
        Translation2d CS_LowerEntrance = FieldConstants.Community.chargingStationCornersBlue[0];
        Translation2d CS_UpperEntrance = FieldConstants.Community.chargingStationCornersBlue[2];

        // Make these Safe
        CS_LowerEntrance = CS_LowerEntrance.plus(new Translation2d(0, -.75));
        CS_UpperEntrance = CS_UpperEntrance.plus(new Translation2d(0, -.75));

        // 4th Game Piece 
        Translation2d Stage4 = FieldConstants.Auto.Waypoints.StagingWaypoints[0]
        .getPose(currentAlliance).getTranslation();

        Translation2d ScoreSpot1 = FieldConstants.getTargetPositionPose(
            TargetedPositions.GRID_9,
            currentAlliance);

        Translation2d ScoreSpot2 = FieldConstants.getTargetPositionPose(
            TargetedPositions.GRID_8,
            currentAlliance);

      // Score the Cone!
      addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT));
      addAction(new WaitAction(.1));
      addAction(new GrasperAction(true));
      addAction(new WaitAction(.1));

      // Tell the Arm to go to the Backside Intake while you slowly drive
      DriveTrajectoryAction scoreToGp1TrajectoryAction = new DriveTrajectoryAction(
        SwerveRotation.FRONT_FACING_GRID.getRotation(), 1.4, 1.8);
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot1,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.addTranslation(CS_LowerEntrance);
        scoreToGp1TrajectoryAction.addTranslation(CS_UpperEntrance);
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                Stage4.plus(new Translation2d(0,0)),
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.setEndVelocity(0);
        scoreToGp1TrajectoryAction.generate();
        incrimentDuration(scoreToGp1TrajectoryAction.getEstimatedDuration());

        // Drive Trajectory and Get Arm to Intake Position (and Open)
        addAction(
            new ParallelAction(
                scoreToGp1TrajectoryAction,
                new SequentialAction(
                    new WaitAction(.0),
                    new ArmAction(ArmTargets.INTAKE_GROUND_BACK),
                    new GrasperAction(true)
                )
            )
        );

        // Cautionary Wait
        addAction(new WaitAction(.1));

        // Drive back!
        DriveTrajectoryAction Gp1ToScoreSpot2 = new DriveTrajectoryAction(
        SwerveRotation.FRONT_FACING_GRID.getRotation(), 2, 1.8);
        Gp1ToScoreSpot2.addPose(
            new Pose2d(
                Stage4.plus(new Translation2d(0,0)),
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Gp1ToScoreSpot2.addTranslation(CS_UpperEntrance);
        Gp1ToScoreSpot2.addTranslation(CS_LowerEntrance);
        Gp1ToScoreSpot2.addPose(
            new Pose2d(
                ScoreSpot2.plus(new Translation2d(-.52,.1)),
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Gp1ToScoreSpot2.setEndVelocity(0);
        Gp1ToScoreSpot2.generate();
       // Gp1ToScoreSpot2.useVision();
        incrimentDuration(Gp1ToScoreSpot2.getEstimatedDuration());

        addAction(
            new ParallelAction(
                Gp1ToScoreSpot2,
                new ArmAction(ArmTargets.TOP_SCORING_FRONT)
            )
        );

        // Wait and Eject
        addAction(new WaitAction(.1));
        addAction(new GrasperAction(true));

        //addAction(new ArmAction(ArmTargets.HOME_BACKSIDE));


    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
    
}
