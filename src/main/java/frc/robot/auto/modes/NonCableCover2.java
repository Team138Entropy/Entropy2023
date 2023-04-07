package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
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
import frc.robot.auto.actions.GrasperTimingAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SequentialAction;
import frc.robot.auto.actions.WaitAction;

public class NonCableCover2 extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();

    // NON Cable Cover 2 High
    public NonCableCover2( )
    {
        // Set Odometry Starting Location
        setStartingPosition(
            TargetedPositions.GRID_1, 
            SwerveRotation.FRONT_FACING_GRID
        );

        // Current Alliance of this Auto Mode
        Alliance currentAlliance = mRobotState.getAlliance();
        currentAlliance = Alliance.Blue; //always blue

        // Scoring Positions
        Translation2d ScoreSpot1 = FieldConstants.getTargetPositionPose(
        TargetedPositions.GRID_1,
        currentAlliance);
         
        Translation2d ScoreSpot2 = FieldConstants.getTargetPositionPose(
         TargetedPositions.GRID_2,
         currentAlliance);
         ScoreSpot2 = ScoreSpot2.plus(new Translation2d(.1,0));

        
        // Game Object Staging Positions (Left to Right just like the Grid)
        Translation2d Stage1 = FieldConstants.Auto.Waypoints.StagingWaypoints[3]
            .getPose(currentAlliance).getTranslation();
        Translation2d Stage2 = FieldConstants.Auto.Waypoints.StagingWaypoints[2]
            .getPose(currentAlliance).getTranslation();  

        // Charging Station Exit and Entrance Points
        Translation2d CS_LowerEntrance = ScoreSpot1.plus(new Translation2d(1, 0));
        Translation2d CS_UpperEntrance = FieldConstants.Community.chargingStationCornersBlue[3];

        // Slightly Translate Points
        //      Blue should have slightly greater Y
        CS_UpperEntrance = CS_UpperEntrance.plus(new Translation2d(-.2,.8));
        
        // Slightly Shifted CS Upper Entrance for Stage Spot 1
        //      this is to ensure the approach for stage 1 is good
        Translation2d CS_UpperEntrance_Tighter = new Translation2d(
            CS_UpperEntrance.getX(), Stage1.getY()

        );

        // Offset First Staging Point to support Straight on Approach
        //      Otherwise it is trying to line up center of robot
        Stage1 = Stage1.plus(new Translation2d(0,0));

      // Score the Cone!
      addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT));
      addAction(new WaitAction(.1));
      addAction(new GrasperAction(true));
      addAction(new WaitAction(.1));

      // Tell the Arm to go to the Backside Intake while you slowly drive
      DriveTrajectoryAction scoreToGp1TrajectoryAction = new DriveTrajectoryAction(
        SwerveRotation.FRONT_FACING_GRID.getRotation(), 1.1, 1.3);
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
                Stage1.plus(new Translation2d(.15,0)),
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
                    new GrasperTimingAction(Constants.Grasper.autoDefaultDelay),
                    new ArmAction(ArmTargets.INTAKE_GROUND_BACK),
                    new GrasperAction(true),
                    new WaitAction(.1)
                )
            )
        );

        // Cautionary Wait
        addAction(new WaitAction(.1));

        // Drive back!
        DriveTrajectoryAction Gp1ToScoreSpot2 = new DriveTrajectoryAction(
        SwerveRotation.FRONT_FACING_GRID.getRotation(), 2, 1.9);
        Gp1ToScoreSpot2.addPose(
            new Pose2d(
                Stage1,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        Gp1ToScoreSpot2.addTranslation(CS_UpperEntrance);
        Gp1ToScoreSpot2.addTranslation(CS_LowerEntrance.plus(new Translation2d(-.15,-.1)));
        Gp1ToScoreSpot2.addPose(
            new Pose2d(
                ScoreSpot2.plus(new Translation2d(-.0,.15)),
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
                new ArmAction(ArmTargets.TOP_SCORING_FRONT_CUBE)
            )
        );

        // Wait and Eject
        addAction(new GrasperTimingAction(Constants.Grasper.regrabSafeDelay));
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
