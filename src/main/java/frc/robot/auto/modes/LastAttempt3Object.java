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

public class LastAttempt3Object extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();

    // Cable Cover 2 High
    public LastAttempt3Object( )
    {
        // Set Odometry Starting Location
        setStartingPosition(
            TargetedPositions.GRID_1, 
            SwerveRotation.BACK_FACING_GRID
        );

        // Current Alliance of this Auto Mode
        Alliance currentAlliance = mRobotState.getAlliance();

        // 1st and 2nd Game Piece
        Translation2d Stage1 = FieldConstants.Auto.Waypoints.StagingWaypoints[3]
            .getPose(currentAlliance).getTranslation();
        Translation2d Stage2 = FieldConstants.Auto.Waypoints.StagingWaypoints[2]
            .getPose(currentAlliance).getTranslation();

        Translation2d ScoreSpot1 = FieldConstants.getTargetPositionPose(
            TargetedPositions.GRID_1,
            currentAlliance);

        Translation2d ScoreSpot2 = FieldConstants.getTargetPositionPose(
            TargetedPositions.GRID_2,
            currentAlliance);

        Translation2d CS_LowerEntrance = ScoreSpot1.plus(new Translation2d(1, 0));
        Translation2d CS_UpperEntrance = FieldConstants.Community.chargingStationCornersBlue[3];

        // Slightly Translate Points
        //      Blue should have slightly greater Y
        CS_UpperEntrance = CS_UpperEntrance.plus(new Translation2d(.5,.8));
        
        // Slightly Shifted CS Upper Entrance for Stage Spot 1
        //      this is to ensure the approach for stage 1 is good
        Translation2d CS_UpperEntrance_Tighter = new Translation2d(
            CS_UpperEntrance.getX(), Stage1.getY()

        );

      // Quickly Score the Cone Low
      addAction(new ArmAction(ArmTargets.LOW_SCORING_BACK));
      addAction(new WaitAction(.1));
      addAction(new GrasperAction(true));
      addAction(new WaitAction(.1));

      // Drive to the Cube
      DriveTrajectoryAction scoreToGp1TrajectoryAction = new DriveTrajectoryAction(
        SwerveRotation.BACK_FACING_GRID.getRotation(), .8, .8);
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot1,
                SwerveRotation.BACK_FACING_GRID.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.addTranslation(CS_UpperEntrance_Tighter);
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                Stage1.plus(new Translation2d(0,0)),
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
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

        // Drive back - Score this High
        DriveTrajectoryAction stage1ToScore2 = new DriveTrajectoryAction(
            SwerveRotation.FRONT_FACING_GRID.getRotation(), .8, .8);
            stage1ToScore2.addPose(
                new Pose2d(
                    Stage1,
                    SwerveRotation.FRONT_FACING_GRID.getRotation()
                )
            );
            stage1ToScore2.addTranslation(CS_UpperEntrance);
            stage1ToScore2.addTranslation(CS_LowerEntrance);
            stage1ToScore2.addPose(
                new Pose2d(
                    ScoreSpot2,
                    SwerveRotation.FRONT_FACING_GRID.getRotation()
                )
            );
            stage1ToScore2.generate();
            incrimentDuration(stage1ToScore2.getEstimatedDuration());

        addAction(
            new ParallelAction(
                stage1ToScore2,
                new SequentialAction(
                    new WaitAction(.0),
                    new ArmAction(ArmTargets.TOP_SCORING_FRONT_CUBE)
                )
            )
        );

        // Score Cube High
        addAction(new GrasperAction(true));
        addAction(new WaitAction(.15));


        /*
         *  var optimalHeading = poses.get(i + 1).getTranslation()
         * .minus(poses.get(i).getTranslation()).getAngle(); 
I believe you can just subtract them from each other (rotation2d.minus)

         */

        // Get Last Game Piece
         DriveTrajectoryAction Score2ToGp2TrajectoryAction = new DriveTrajectoryAction(
            SwerveRotation.FRONT_FACING_GRID.getRotation(), .8, .8);
        Score2ToGp2TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot2,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Score2ToGp2TrajectoryAction.addTranslation(CS_LowerEntrance);
        Score2ToGp2TrajectoryAction.addTranslation(CS_UpperEntrance);

        // Calculate Heading to Get Robot to Face Node
       // var optimalHeading = CS_UpperEntrance.minus(Stage2).getAngle(); 
        var optimalHeading = Stage2.minus(CS_UpperEntrance).getAngle();
        
        Score2ToGp2TrajectoryAction.addPose(
            new Pose2d(
                Stage2.plus(new Translation2d(0, -.1)),
                optimalHeading
            )
        );
        
        Score2ToGp2TrajectoryAction.generate();
        //addAction(Score2ToGp2TrajectoryAction);
        incrimentDuration(Score2ToGp2TrajectoryAction.getEstimatedDuration());

        addAction(
            new ParallelAction(
                Score2ToGp2TrajectoryAction,
                new SequentialAction(
                    new WaitAction(.5),
                    new ArmAction(ArmTargets.INTAKE_GROUND_FRONT)
                )
            )
        );

        // Cautionary  Wait
        addAction(new WaitAction(.1));

        // Now Drive 2 More Trajectories
        //  1st one is still odometry based, the second one is vision based

        DriveTrajectoryAction Gp2ToScore1Part1 = new DriveTrajectoryAction(
            optimalHeading, .8, .8);
        Gp2ToScore1Part1.addPose(
            new Pose2d(
                Stage2.plus(new Translation2d(0, -.1)),
                optimalHeading
            )
        );
        Gp2ToScore1Part1.addTranslation(CS_UpperEntrance);

        Gp2ToScore1Part1.addPose(
            new Pose2d(
                CS_LowerEntrance,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        
        Gp2ToScore1Part1.generate();
        incrimentDuration(Gp2ToScore1Part1.getEstimatedDuration());

        
        DriveTrajectoryAction Gp2ToScore1Part2 = new DriveTrajectoryAction(
            optimalHeading, .8, .8);
        Gp2ToScore1Part2.addPose(
            new Pose2d(
                CS_LowerEntrance,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Gp2ToScore1Part2.addPose(
            new Pose2d(
                ScoreSpot1,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Gp2ToScore1Part2.generate();
        Gp2ToScore1Part2.useVision();
        incrimentDuration(Gp2ToScore1Part2.getEstimatedDuration());

        addAction(
            new ParallelAction(
                new ArmAction(ArmTargets.TOP_SCORING_FRONT),
                new SequentialAction(
                    Gp2ToScore1Part1,
                    Gp2ToScore1Part2
                )
            )
        );

        // Wait to Drop
        addAction(new WaitAction(.1));
        addAction(new GrasperAction(true));









    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
    
}
