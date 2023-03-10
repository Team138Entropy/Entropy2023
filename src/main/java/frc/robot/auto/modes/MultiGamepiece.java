package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

public class MultiGamepiece extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();

    public MultiGamepiece()
    {
        /*
         * 
         */
        // Is Blue or Red Alliance
        boolean isBlueAlliance = (mRobotState.getAlliance() == Alliance.Blue);

        // How Many Stages to do?
        //int StageCount = targetGamepieces.length;
        int CurrentStage = 0;
        GamePiece Gamepiece1 = GamePiece.Cone;
        GamePiece Gamepiece2 = GamePiece.Cube;
        GamePiece Gamepiece3 = GamePiece.Cube;    

        // Set Odometry Starting Location
        setStartingPosition(
            (isBlueAlliance) ? TargetedPositions.GRID_1 : TargetedPositions.GRID_9, 
            SwerveRotation.FRONT_FACING_GRID
        );

        // Scoring Positions
        Translation2d ScoreSpot1 = FieldConstants.getTargetPositionPose(TargetedPositions.GRID_1,
             mRobotState.getAlliance());
        Translation2d ScoreSpot2 = FieldConstants.getTargetPositionPose(TargetedPositions.GRID_2,
             mRobotState.getAlliance());
        Translation2d ScoreSpot3 = FieldConstants.getTargetPositionPose(TargetedPositions.GRID_3,
             mRobotState.getAlliance());

        // Game Object Staging Positions (Left to Right just like the Grid)
        Translation2d Stage1 = FieldConstants.Auto.Waypoints.StagingWaypoints[3]
            .getPose(mRobotState.getAlliance()).getTranslation();
        Translation2d Stage2 = FieldConstants.Auto.Waypoints.StagingWaypoints[2]
            .getPose(mRobotState.getAlliance()).getTranslation();  

        // Staging Entrance Points
        Translation2d Stage1Entrance = Stage1.plus(new Translation2d(0, .5));
        Translation2d Stage2Entrance = Stage2.plus(new Translation2d(0, .5));

        // Charging Station Exit and Entrance Points
        Translation2d CS_LowerEntrance = ScoreSpot1.plus(new Translation2d(1, 0));
        Translation2d CS_UpperEntrance = FieldConstants.Community.chargingStationCornersBlue[3];

        // Slightly Translate Points
        //      Blue should have slightly greater Y
        CS_UpperEntrance = CS_UpperEntrance.plus(new Translation2d(1, isBlueAlliance ? 1 : -1.5));


        // 1. Score First Object (Starts with Robot)
        if(GamePiece.Cone == Gamepiece1)
        {
            addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT));
        }else
        {
            addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT_CUBE));
        }
        addAction(new WaitAction(.2));
        addAction(new GrasperAction(true));


        // 2. Drive to Game Piece 2 (keep arm on)
        DriveTrajectoryAction scoreToGp1TrajectoryAction = new DriveTrajectoryAction(
                SwerveRotation.FRONT_FACING_GRID.getRotation(),.8,2);
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot1,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                CS_UpperEntrance,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                Stage1Entrance,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                Stage1,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.generate();

        // Drive Trajectory and Get Arm to Intake Position (and Open)
        addAction(
            new ParallelAction(
                scoreToGp1TrajectoryAction,
                new SequentialAction(
                    new WaitAction(.25),
                    new ArmAction(ArmTargets.INTAKE_GROUND_FRONT),
                    new GrasperAction(true)
                )
            )
        );

        // 3. Drive to Score 2 (keep arm on)
        DriveTrajectoryAction gp1ToScore2TrajectoryAction = new DriveTrajectoryAction(
                SwerveRotation.FRONT_FACING_RIGHT.getRotation(),.8,2);
        gp1ToScore2TrajectoryAction.addPose(
            new Pose2d(
                Stage1,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );
        gp1ToScore2TrajectoryAction.addPose(
            new Pose2d(
                CS_UpperEntrance,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        gp1ToScore2TrajectoryAction.addPose(
            new Pose2d(
                CS_LowerEntrance,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        gp1ToScore2TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot2,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        gp1ToScore2TrajectoryAction.generate();
        addAction(
            new ParallelAction(
                gp1ToScore2TrajectoryAction,
                new SequentialAction(
                    new WaitAction(.2),
                    new ArmAction(ArmTargets.LOW_SCORING_FRONT)
                )
            )
        );

        // Score Game GP 2
        addAction(new WaitAction(.1));
        addAction(new GrasperAction(true));

        // 4. Drive to GP 3
        DriveTrajectoryAction Score2ToGp2TrajectoryAction = new DriveTrajectoryAction(
            SwerveRotation.FRONT_FACING_GRID.getRotation(),.8,2);
        Score2ToGp2TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot2,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Score2ToGp2TrajectoryAction.addPose(
            new Pose2d(
                CS_LowerEntrance,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Score2ToGp2TrajectoryAction.addPose(
            new Pose2d(
                CS_UpperEntrance,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );        
        Score2ToGp2TrajectoryAction.addPose(
            new Pose2d(
                Stage2Entrance,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );

        Score2ToGp2TrajectoryAction.addPose(
            new Pose2d(
                Stage2,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );
        Score2ToGp2TrajectoryAction.generate();
        addAction(
            new ParallelAction(
                Score2ToGp2TrajectoryAction,
                new SequentialAction(
                    new WaitAction(.25),
                    new ArmAction(ArmTargets.INTAKE_GROUND_FRONT),
                    new GrasperAction(true)
                )
            )
        );

        // Drive to Score 3
        DriveTrajectoryAction Gp2ToScore3TrajectoryAction = new DriveTrajectoryAction(
            SwerveRotation.FRONT_FACING_RIGHT.getRotation(),.8,2);
        Gp2ToScore3TrajectoryAction.addPose(
            new Pose2d(
                Stage2,
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );
        Gp2ToScore3TrajectoryAction.addPose(
            new Pose2d(
                CS_UpperEntrance,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        ); 
        Gp2ToScore3TrajectoryAction.addPose(
            new Pose2d(
                CS_LowerEntrance,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        ); 
        Gp2ToScore3TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot3,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Gp2ToScore3TrajectoryAction.generate();
        addAction(
            new ParallelAction(
                Gp2ToScore3TrajectoryAction,
                new SequentialAction(
                    new WaitAction(.2),
                    new ArmAction(ArmTargets.LOW_SCORING_FRONT)
                )
            )
        );

        // Score Game GP 3
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
