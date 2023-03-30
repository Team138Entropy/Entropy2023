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

public class MultiGamepiece extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();

    // MultiGamepiece
    //  This mode will do 1, 2 or 3 game piece autos
    //  It will score high, mid or low for each position
    //  Unfortunatly high auto contains some time constraints
    public MultiGamepiece(Enums.ScorePositions[] scorePos, Enums.GamePiece[] gamePieces )
    {
        // How Many Gamepieces to Score?
        int ScoreCount = scorePos.length;

        // What Stages to do
        boolean doStage1 = (1 <= ScoreCount);
        boolean doStage2 = (2 <= ScoreCount);
        boolean doStage3 = (3 <= ScoreCount);

        // What object to do at each stage (Default to Cube)
        int GamepieceCount = gamePieces.length;
        GamePiece Gamepiece1 = (GamepieceCount >= 1) ? gamePieces[0] : GamePiece.Cube;
        GamePiece Gamepiece2 = (GamepieceCount >= 2) ? gamePieces[1] : GamePiece.Cube;
        GamePiece Gamepiece3 = (GamepieceCount >= 3) ? gamePieces[2] : GamePiece.Cube;
   
        // Is Blue or Red Alliance
        //boolean isBlueAlliance = (mRobotState.getAlliance() == Alliance.Blue);

        // Speed Configurations 
        //   [Gamepiece Count][Each Level][Velocity, Acceleration]
        double[][][] TrajSpeedConfig = new double[3][4][2];

        // 1 Object Mode Speeds (Drives 1 Trajectory)
        TrajSpeedConfig[0][0][0] = 1.9; // Traj 1 Vel
        TrajSpeedConfig[0][0][1] = 2;   // Traj 1 Accel
        TrajSpeedConfig[0][1][0] = 0; // Traj 2 Vel
        TrajSpeedConfig[0][1][1] = 0;   // Traj 2 Accel
        TrajSpeedConfig[0][2][0] = 0; // Traj 3 Vel
        TrajSpeedConfig[0][2][1] = 0;   // Traj 3 Accel
        TrajSpeedConfig[0][3][0] = 0; // Traj 4 Vel
        TrajSpeedConfig[0][3][1] = 0;   // Traj 4 Accel

        // 2 Object Mode Speeds (Drives 3 Trajectories)
        // TODO (these are purposefully slow for now)
        TrajSpeedConfig[1][0][0] = 1.2; // Traj 1 Vel
        TrajSpeedConfig[1][0][1] = 1.2;   // Traj 1 Accel
        TrajSpeedConfig[1][1][0] = 1.8; // Traj 2 Vel
        TrajSpeedConfig[1][1][1] = 1.9;   // Traj 2 Accel
        TrajSpeedConfig[1][2][0] = 1.3; // Traj 3 Vel
        TrajSpeedConfig[1][2][1] = 1.7;   // Traj 3 Accel
        TrajSpeedConfig[1][3][0] = 0; // Traj 4 Vel
        TrajSpeedConfig[1][3][1] = 0;   // Traj 4 Accel

        // 3 Object Mode Speeds (Drives 4 Trajectories)
        TrajSpeedConfig[2][0][0] = 1.4; // Traj 1 Vel
        TrajSpeedConfig[2][0][1] = 1.4;   // Traj 1 Accel
        TrajSpeedConfig[2][1][0] = 1.4; // Traj 2 Vel
        TrajSpeedConfig[2][1][1] = 1.4;   // Traj 2 Accel
        TrajSpeedConfig[2][2][0] = 1.4; // Traj 3 Vel
        TrajSpeedConfig[2][2][1] = 1.4;   // Traj 3 Accel
        TrajSpeedConfig[2][3][0] = 1.4; // Traj 4 Vel
        TrajSpeedConfig[2][3][1] = 1.4;   // Traj 4 Accel

        // Selected Speed Profile (However Many Score Positions are specififed)
        var CurSpeedConfig = TrajSpeedConfig[ScoreCount - 1];

        // Starting Position
        SwerveRotation startingRotation = SwerveRotation.BACK_FACING_GRID;
        if(GamePiece.Cone == Gamepiece1)
        {
            startingRotation = SwerveRotation.FRONT_FACING_GRID;
        }

        // Second Scoring Rotation will probably be the same as first
        // leave it like this for now
        SwerveRotation secondRotation = startingRotation;
        SwerveRotation thirdRotation = startingRotation;

        // Set Odometry Starting Location
        setStartingPosition(
            TargetedPositions.GRID_1, 
            startingRotation
        );

        // Force Blue Alliance now - update maybe no longer do this?
        // the Red alliance post flipper handles the rest
        Alliance currentAlliance = mRobotState.getAlliance();
        //currentAlliance = Alliance.Blue; // everything should be set

        // Scoring Positions
        Translation2d ScoreSpot1 = FieldConstants.getTargetPositionPose(
            TargetedPositions.GRID_1,
            currentAlliance);
             
        Translation2d ScoreSpot2 = FieldConstants.getTargetPositionPose(
             TargetedPositions.GRID_2,
             currentAlliance);
             ScoreSpot2 = ScoreSpot2.plus(new Translation2d(.0,0));
        Translation2d ScoreSpot3 = FieldConstants.getTargetPositionPose(
            TargetedPositions.GRID_3,
            currentAlliance);
             ScoreSpot3 = ScoreSpot3.plus(new Translation2d(.2,0));

        // Game Object Staging Positions (Left to Right just like the Grid)
        Translation2d Stage1 = FieldConstants.Auto.Waypoints.StagingWaypoints[3]
            .getPose(currentAlliance).getTranslation();
        Translation2d Stage2 = FieldConstants.Auto.Waypoints.StagingWaypoints[2]
            .getPose(currentAlliance).getTranslation();  

        // Offset First Staging Point to support Straight on Approach
        //      Otherwise it is trying to line up center of robot
        Stage1 = Stage1.plus(new Translation2d(-.7,0));

        // Staging Entrance Points
        Translation2d Stage2Entrance = Stage2.plus(new Translation2d(-.3,1.2));

        // Charging Station Exit and Entrance Points
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


        // Score Gamepiece 1
        if(doStage1)
        {
            // 1. Score First Object (Starts with Robot)
            if(GamePiece.Cone == Gamepiece1)
            {
                addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT));
            }else
            {
                addAction(new ArmAction(ArmTargets.LOW_SCORING_BACK));
            }
            addAction(new WaitAction(.1));
            addAction(new GrasperAction(true));


            // 2. Drive to Game Piece 2 (keep arm on)
            DriveTrajectoryAction scoreToGp1TrajectoryAction = new DriveTrajectoryAction(
                    startingRotation.getRotation(),CurSpeedConfig[0][0],CurSpeedConfig[0][1]);
            scoreToGp1TrajectoryAction.addPose(
                new Pose2d(
                    ScoreSpot1,
                    startingRotation.getRotation()
                )
            );
            scoreToGp1TrajectoryAction.addDifferentialPose( // Differntial Pose does not care about rotation
                new Pose2d(
                    CS_UpperEntrance_Tighter,
                    SwerveRotation.BACK_FACING_GRID.getRotation()
                )
            );
            scoreToGp1TrajectoryAction.addPose(
                new Pose2d(
                    Stage1,
                    SwerveRotation.BACK_FACING_GRID.getRotation()
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
                        new GrasperAction(true),
                        new ArmAction(ArmTargets.INTAKE_GROUND_FRONT_CUBE)
                    )
                )
            );
        }

        // Score Gamepiece 2
        if(doStage2)
        {
            // 3. Drive to Score 2 (keep arm on)
            DriveTrajectoryAction gp1ToScore2TrajectoryAction = new DriveTrajectoryAction(
                    SwerveRotation.BACK_FACING_GRID.getRotation(),CurSpeedConfig[1][0],CurSpeedConfig[1][1]);
            gp1ToScore2TrajectoryAction.addPose(
                new Pose2d(
                    Stage1,
                    SwerveRotation.BACK_FACING_GRID.getRotation()
                )
            );
            gp1ToScore2TrajectoryAction.addDifferentialPose(
                new Pose2d(
                    CS_UpperEntrance,
                    startingRotation.getRotation()
                )
            );
            gp1ToScore2TrajectoryAction.addDifferentialPose(
                new Pose2d(
                    CS_LowerEntrance,
                    startingRotation.getRotation()
                )
            );
            gp1ToScore2TrajectoryAction.addPose(
                new Pose2d(
                    ScoreSpot2.plus(new Translation2d(0,0)),
                    startingRotation.getRotation()
                )
            );
            gp1ToScore2TrajectoryAction.generate();
            incrimentDuration(gp1ToScore2TrajectoryAction.getEstimatedDuration());
            addAction(
                new ParallelAction(
                    gp1ToScore2TrajectoryAction,
                    new SequentialAction(
                        new WaitAction(.0),
                        new ArmAction(ArmTargets.MID_SCORING_FRONT)
                    )
                )
            );

            // Score Top
            // TODO - this should be more dynamic
            addAction(new ArmAction(ArmTargets.TOP_SCORING_FRONT_CUBE));

            // Score Game GP 2
            addAction(new WaitAction(.0));
            addAction(new GrasperAction(true));

            // 4. Drive to GP 3
            DriveTrajectoryAction Score2ToGp2TrajectoryAction = new DriveTrajectoryAction(
                startingRotation.getRotation(),CurSpeedConfig[2][0],CurSpeedConfig[2][1]);
            Score2ToGp2TrajectoryAction.addPose(
                new Pose2d(
                    ScoreSpot2,
                    startingRotation.getRotation()
                )
            );
            Score2ToGp2TrajectoryAction.addPose(
                new Pose2d(
                    CS_LowerEntrance,
                    startingRotation.getRotation()
                )
            );
            /* 
            Score2ToGp2TrajectoryAction.addPose(
                new Pose2d(
                    CS_UpperEntrance,
                    startingRotation.getRotation()
                )
            );   
            */     
            Score2ToGp2TrajectoryAction.addDifferentialPose(
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
            //addAction(Score2ToGp2TrajectoryAction);
            incrimentDuration(Score2ToGp2TrajectoryAction.getEstimatedDuration());
            
            addAction(
                new ParallelAction(
                    Score2ToGp2TrajectoryAction,
                    new SequentialAction(
                        new WaitAction(.25),
                        new ArmAction(ArmTargets.INTAKE_GROUND_FRONT)
                    )
                )
            );
            addAction(new WaitAction(.2));
            addAction(new GrasperAction(false));
            addAction(new ArmAction(ArmTargets.HOME_BACKSIDE));
        }

        // Score Gamepiece 3
        if(doStage3)
        {
            // Drive to Score 3
            DriveTrajectoryAction Gp2ToScore3TrajectoryAction = new DriveTrajectoryAction(
                SwerveRotation.FRONT_FACING_RIGHT.getRotation(),CurSpeedConfig[3][0],CurSpeedConfig[3][1]);
            Gp2ToScore3TrajectoryAction.addPose(
                new Pose2d(
                    Stage2,
                    SwerveRotation.FRONT_FACING_RIGHT.getRotation()
                )
            );
            Gp2ToScore3TrajectoryAction.addDifferentialPose(
                new Pose2d(
                    CS_UpperEntrance,
                    SwerveRotation.FRONT_FACING_GRID.getRotation()
                )
            ); 
            Gp2ToScore3TrajectoryAction.addDifferentialPose(
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
            incrimentDuration(Gp2ToScore3TrajectoryAction.getEstimatedDuration());
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
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  mAutoActions )  {
        runAction(currentAction);
     }
    }
    
}
