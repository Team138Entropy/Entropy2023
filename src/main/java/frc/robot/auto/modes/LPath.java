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

public class LPath extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();

    // Cable Cover 2 High
    public LPath( )
    {
        // Set Odometry Starting Location
        setStartingPosition(
            TargetedPositions.GRID_1, 
            SwerveRotation.FRONT_FACING_GRID
        );

        // Current Alliance of this Auto Mode
        Alliance currentAlliance = mRobotState.getAlliance();
        currentAlliance = Alliance.Blue;

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
            TargetedPositions.GRID_1,
            currentAlliance);

        Translation2d ScoreSpot2 = FieldConstants.getTargetPositionPose(
            TargetedPositions.GRID_8,
            currentAlliance);



      // Tell the Arm to go to the Backside Intake while you slowly drive
      DriveTrajectoryAction scoreToGp1TrajectoryAction = new DriveTrajectoryAction(
        SwerveRotation.FRONT_FACING_GRID.getRotation(), .6, .6);
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot1,
                SwerveRotation.FRONT_FACING_GRID.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.addTranslation(ScoreSpot1.plus(new Translation2d(1.5, 0)));
        scoreToGp1TrajectoryAction.addPose(
            new Pose2d(
                ScoreSpot1.plus(new Translation2d(1.5, -1)),
                SwerveRotation.FRONT_FACING_RIGHT.getRotation()
            )
        );
        scoreToGp1TrajectoryAction.setEndVelocity(0);
        scoreToGp1TrajectoryAction.generate();
        incrimentDuration(scoreToGp1TrajectoryAction.getEstimatedDuration());

        // Drive Trajectory and Get Arm to Intake Position (and Open)
        addAction(
            new ParallelAction(
                scoreToGp1TrajectoryAction
            )
        );

        

    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
    
}
