package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.Enums.ArmTargets;
import frc.robot.Enums.SwerveRotation;
import frc.robot.Enums.TargetedPositions;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.Waypoint;
import frc.robot.vision.AutoPilot;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.List;
import java.util.Vector;

// DriveTrajectoryAction 
public class DriveTrajectoryAction implements Action {
    private final Drive mDrive = Drive.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private double mMaxVelocityMS = 2;
    private double mMaxVAccelerationMSS = 8;
    private double mMaxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);
    private Timer mTimer;

    private final PIDController mXController = new PIDController(2.5, 0.0, 0.0);
    private final PIDController mYController = new PIDController(2.5, 0.0, 0.0);
    private final ProfiledPIDController mThetaController =
    new ProfiledPIDController(
        7, 0.0, 0.0, new TrapezoidProfile.Constraints(8, 0.8));

    private final HolonomicDriveController mHolonomicDriveController = new HolonomicDriveController(mXController, 
        mYController, mThetaController);
    private List<Pose2d> mPoses;
    private Trajectory mTrajectory = null;
    private Rotation2d mOrentation;


    public DriveTrajectoryAction(Rotation2d orentation, double velocity, double acceleration)
    {
        mOrentation = orentation;
        mMaxVelocityMS = velocity;
        mMaxVAccelerationMSS = acceleration;
        init();
    }

    public DriveTrajectoryAction(Rotation2d orentation)
    {
        mOrentation = orentation;
        init();
    }

    private void init()
    {
        mPoses = new Vector<>();
        mThetaController.enableContinuousInput(-Math.PI, Math.PI);


    }

    public void addPose(Pose2d pose)
    {
        mPoses.add(pose);
    }
    
    public void addWaypoint(Waypoint wp)
    {
        Pose2d posePoint = new Pose2d(
            wp.getPose(mRobotState.getAlliance()).getTranslation(),
            wp.getPose(mRobotState.getAlliance()).getRotation()
        );
        mPoses.add(posePoint);
    }

    public void addWaypoint(Waypoint wp, SwerveRotation rotation)
    {
        Pose2d posePoint = new Pose2d(
            wp.getPose(mRobotState.getAlliance()).getTranslation(),
            rotation.getRotation()
        );
        mPoses.add(posePoint);
    }
    
    // Generate the Trajectory
    public void generate()
    {
        TrajectoryConfig config = new TrajectoryConfig( //Vel, Accel
            mMaxVelocityMS, mMaxVAccelerationMSS)
            .setKinematics(mDrive.getSwerveKinematics())
            .setStartVelocity(0)
            .setEndVelocity(0).addConstraint(
                new CentripetalAccelerationConstraint(mMaxCentripetalAccelerationMetersPerSec2)
        );

        mTrajectory = TrajectoryGenerator.generateTrajectory(mPoses, config);
    }

    @Override
    public void start()
    {
        // Reset PID Controllers
        mXController.reset();
        mYController.reset();
        mThetaController.reset(null);

        // Initialize the timer.
        mTimer = new Timer();
        mTimer.start();

    }

    @Override 
    public void update()
    {
        // Verify a Trajectory is had
        if(mTrajectory == null) return;

        // Sample the Trajectory through the total time
        if(mTimer.get() < mTrajectory.getTotalTimeSeconds())
        {
            State currentTrajectoryState = mTrajectory.sample(mTimer.get());

            //mHolonomicDriveController.calculate(mEndPose, mEndPose, mMaxVelocityMS, null)
            ChassisSpeeds calculatedSpeeds = mHolonomicDriveController.calculate(
                mRobotState.getPose(), currentTrajectoryState, mOrentation
            );

            // Call Autonomous Chasis Speed 
            var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

            // Desaturate wheel speeds - keeps speed below a maximum speed
            SwerveDriveKinematics.desaturateWheelSpeeds(targetSwerveModuleStates, 2);
                
            // Set Swerve to those Module States
            mDrive.setModuleStates(targetSwerveModuleStates);

        }
    }

    @Override
    public void done() {
        // Stop the Swerve Drive System
        mDrive.setSwerveDrive(new Translation2d(), 0, true, true, false);

    }

    @Override 
    public boolean isFinished() {
        return mTimer.get() >= mTrajectory.getTotalTimeSeconds();
    }
}
