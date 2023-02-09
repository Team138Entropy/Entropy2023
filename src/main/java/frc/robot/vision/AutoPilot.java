package frc.robot.vision;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.auto.TrajectoryFollower;
import frc.robot.subsystems.Drive;
import frc.robot.util.TuneableNumber;

// Auto Pilot System
//      Calculates How to Drive to a Pose
//      https://github.com/STMARobotics/swerve-test/blob/main/src/main/java/frc/robot/commands/ChaseTagCommand.java
//      Todo: Need to figure out how to allow constraints in this system.. path solving variables maybe A*
//      Todo: Regineration of Path, especially if noin new Latent vision targets
//     
//      https://www.geeksforgeeks.org/a-search-algorithm/ ? Gray Zones?
//      https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/commands/HoldPose.java
//      https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/PoseEstimator.java
public class AutoPilot {
    private static AutoPilot mInstance;

    public static synchronized AutoPilot getInstance() {
        if (mInstance == null) {
          mInstance = new AutoPilot();
        }
        return mInstance;
    }

    

    private final Drive mDrive = Drive.getInstance();
    private final TrajectoryFollower mTrajectoryFollower = TrajectoryFollower.getInstance();
    private boolean mRunning;
    private Trajectory mTrajectory;
    private boolean mTrajectorySet;
    private Pose2d mStartingPose;
    private Pose2d mTargetedPose;
    private Pose2d mGoalPose;

    // Current Robot Pose
    private Pose2d mRobotPose;

    // Motion Control
    private static final TrapezoidProfile.Constraints mX_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints mY_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints mOMEGA_CONSTRAINTS =   
            new TrapezoidProfile.Constraints(8, 8);
    private final ProfiledPIDController mXController = new ProfiledPIDController(3, 0, 0, mX_CONSTRAINTS);
    private final ProfiledPIDController mYController = new ProfiledPIDController(3, 0, 0, mY_CONSTRAINTS);
    private final ProfiledPIDController mOmegaController = new ProfiledPIDController(2, 0, 0, mOMEGA_CONSTRAINTS);

    // Tolerances
    private final TuneableNumber mXTolerance = new TuneableNumber("X Tolerance", .2);
    private final TuneableNumber mYTolerance = new TuneableNumber("Y Tolerance", .2);
    private final TuneableNumber mRotationTolerance = new TuneableNumber("Rotation Tolerance", Units.degreesToRadians(3));

    // Type of System being used to Drive
    enum AutoPilotMode {
        TrajectoryFollower,
        MotionControl,
        HoldPose
    };
    private AutoPilotMode mDriveMode;

    private AutoPilot()
    {
        init();
    }

    private void init()
    {
        // Not Currently Running
        mRunning = false;
        mTrajectorySet = false;

        // Drive Style
        mDriveMode = AutoPilotMode.HoldPose;

        // Placeholder Pose Targets
        mTargetedPose = new Pose2d();
        mStartingPose = new Pose2d();
        mGoalPose = new Pose2d();
        mRobotPose = new Pose2d();

        // Motion Control Init
        mXController.setTolerance(0.2);
        mYController.setTolerance(0.2);
        mOmegaController.setTolerance(Units.degreesToRadians(3));
        mOmegaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Set Target Pose to Drive To
    public void setTargetPose(Pose2d targPose)
    {
        mTargetedPose = targPose;
    }

    // Set Pose to Drive From
    public void setStartingPose(Pose2d startPose)
    {
        mStartingPose = startPose;
    }

    // Set Current Robot Pose
    public void setRobotPose(Pose2d currentRobotPose)
    {
        mRobotPose = currentRobotPose;
    }

    // Deprecated?
    private void start()
    {
        // Not Currently Running!
        mRunning = true;

        // Proceed based on Type
        if(AutoPilotMode.TrajectoryFollower == mDriveMode)
        {
            // Generate a Trajectory to Drive
            mTrajectory = TrajectoryGenerator.generateTrajectory(
                    List.of(mStartingPose, mTargetedPose),
                    Constants.SwerveConstants.AutoConstants.slowSwerveSpeedConfig
            );
            mTrajectorySet = true;

            // Set Trajectory to Follow
            mTrajectoryFollower.setTrajectory(mTrajectory);

            // Start Trajectory Folling
            mTrajectoryFollower.Start();
        } else if(AutoPilotMode.MotionControl == mDriveMode)
        {
            // TODO:
            mGoalPose = mTargetedPose;

            // Reset Pose to the Start Pose
            mDrive.resetOdometry(mStartingPose);

            // Reset Positions
            mXController.reset(mStartingPose.getX());
            mYController.reset(mStartingPose.getY());
            mOmegaController.reset(mGoalPose.getRotation().getRadians());


            // Set PID Controler Goals
            mXController.setGoal(mGoalPose.getX());
            mYController.setGoal(mGoalPose.getY());
            mOmegaController.setGoal(mGoalPose.getRotation().getRadians());

            /*
             TODO: Need to generate a list of Poses to Drive
              
             
             */
        } else if(AutoPilotMode.HoldPose == mDriveMode)
        {
            mGoalPose = mTargetedPose;

        }
    }

    // Deprecated?
    public void update() 
    {
        // Start if not started
        if(!mRunning)
        {
            start();
        }

        // Proceed based on Type
        if(AutoPilotMode.TrajectoryFollower == mDriveMode)
        {
            // Keep Following the Trajectory
            mTrajectoryFollower.Update();
        } else if(AutoPilotMode.MotionControl == mDriveMode)
        {
            // Get Robots Current Pose and Calculate the Swerve Speeds
            Pose2d currPose = mDrive.getPose();

            // Calculate Speeds to Reach Goal
            double xSpeed = !mXController.atGoal() ? mXController.calculate(currPose.getX()) : 0;
            double ySpeed = !mYController.atGoal() ? mYController.calculate(currPose.getY()) : 0;
            double omegaSpeed = !mOmegaController.atGoal() ? mOmegaController.calculate(currPose.getRotation().getRadians()) : 0;

            // Set Speeds into Swerve System
            ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);

            // Call Autonomous Chasis Speed 
            var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

            // Set Swerve to those Module States
            mDrive.setModuleStates(targetSwerveModuleStates);
        } else if(AutoPilotMode.HoldPose == mDriveMode)
        {
            // Robot Pose will be constantly updated
            // Calculate Distances between target and robot
            double xDistance = Math.abs(mRobotPose.getX() - mTargetedPose.getX());
            double yDistance = Math.abs(mRobotPose.getY() - mTargetedPose.getY());
            double rotationDistance = Math.abs(mRobotPose.getRotation().getRadians() - mTargetedPose.getRotation().getRadians());

            // Speeds
            double xSpeed = 0;
            double ySpeed = 0;
            double omegaSpeed = 0;

            // Update X if outside Tolerance
            if(xDistance > mXTolerance.get())
            {
                xSpeed = mXController.calculate(mRobotPose.getX(), mTargetedPose.getX());
            }

            // Update Y if outside Tolerance
            if(yDistance > mYTolerance.get())
            {
                ySpeed = mYController.calculate(mRobotPose.getY(), mTargetedPose.getY());
            }

            // Update Rotation if outside Tolerance
            if(rotationDistance > mRotationTolerance.get())
            {
                omegaSpeed = mOmegaController.calculate(mRobotPose.getRotation().getRadians(), mTargetedPose.getRotation().getRadians());
            }
        
            // Set Speeds into Swerve System
            ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);

            // Call Autonomous Chasis Speed 
            var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

            // Set Swerve to those Module States
            mDrive.setModuleStates(targetSwerveModuleStates);
        }
    }

    // Deprecated?
    public void stop()
    {
        mRunning = false;
        mTrajectorySet = false;

        // Proceed based on Type
        if(AutoPilotMode.TrajectoryFollower == mDriveMode)
        {
            // Stop the Trajectory Follower
            mTrajectoryFollower.Stop();
        } else if(AutoPilotMode.MotionControl == mDriveMode)
        {

        }
    }

    // targetPose
    // constantly targets the pose compared to the robot pose
    public void targetPose(Pose2d targetPose)
    {
        // Store targeted Pose for Logging Purposes
        mTargetedPose = targetPose; 

        // Set Goals into PID Controller
        mXController.setGoal(targetPose.getX());
        mYController.setGoal(targetPose.getY());
        mOmegaController.setGoal(targetPose.getRotation().getRadians());

        

        // Calculate Speeds based on Robot Pose
        double xSpeed = !mXController.atGoal() ? mXController.calculate(mRobotPose.getX()) : 0;
        double ySpeed = !mYController.atGoal() ? mYController.calculate(mRobotPose.getY()) : 0;
        double omegaSpeed = !mOmegaController.atGoal() ? mOmegaController.calculate(mRobotPose.getRotation().getRadians()) : 0;

        // Set Speeds into Swerve System
        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);

        // Call Autonomous Chasis Speed 
        var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

        // Set Swerve to those Module States
        mDrive.setModuleStates(targetSwerveModuleStates);     
    }

    public boolean getRunning()
    {
        return mRunning;
    }

    public Trajectory getTrajectory()
    {
        return mTrajectorySet ? mTrajectory : null;
    }

    public void updateSmartDashBoard()
    {
        final String key = "AutoPilot/";
        SmartDashboard.putBoolean(key + "Running", mRunning);
        SmartDashboard.putString(key + "Mode", mDriveMode.toString());
        SmartDashboard.putBoolean(key + "Has Trajectory", mTrajectorySet);
        SmartDashboard.putString(key + "Trajectory Starting Pose", 
            mTrajectorySet ? mTrajectory.getInitialPose().toString() : ""
        );
        SmartDashboard.putString(key + "Target Pose", mTargetedPose.toString());
        SmartDashboard.putString(key + "Starting Pose", mStartingPose.toString());
        SmartDashboard.putString(key + "Goal Pose", mGoalPose.toString());
        
    }
}
