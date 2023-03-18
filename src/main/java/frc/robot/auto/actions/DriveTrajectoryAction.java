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
import frc.robot.util.trajectory.CustomHolonomicDriveController;
import frc.robot.util.trajectory.CustomTrajectoryGenerator;
import frc.robot.util.trajectory.RedAllianceFlipUtility;
import frc.robot.util.trajectory.RotationSequence;
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
    private double mMaxVelocityMS = 1;
    private double mMaxVAccelerationMSS = 4;
    private double mMaxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);
    private Timer mTimer;

    private final PIDController mXController = new PIDController(2.5, 0.0, 0.0);
    private final PIDController mYController = new PIDController(2.5, 0.0, 0.0);
    private final PIDController mThetaController = new PIDController(
        7, 0.0, 0.0);

    // Trajectory Genrator, Drive Controller
    private final CustomTrajectoryGenerator mCustomTrajectoryGenerator = new CustomTrajectoryGenerator();
    private final CustomHolonomicDriveController mCustomHolonomicDriveController = new CustomHolonomicDriveController(
        mXController, mYController, mThetaController
    );

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

    public DriveTrajectoryAction(Rotation2d orentation, double velocity, double acceleration, double centripAccel)
    {
        mOrentation = orentation;
        mMaxVelocityMS = velocity;
        mMaxVAccelerationMSS = acceleration;
        mMaxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(centripAccel);
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

        // Create Generator and Generate
        mCustomTrajectoryGenerator.generateWithPoses(config, mPoses);

        // Store Trajectory
        mTrajectory = mCustomTrajectoryGenerator.getDriveTrajectory();
    }

    @Override
    public void start()
    {
        // Reset PID Controllers
        mXController.reset();
        mYController.reset();
        mThetaController.reset();

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
        double currentTime = mTimer.get();
        if(currentTime < mTrajectory.getTotalTimeSeconds())
        {
            // Get setpoint
            // Flip the trajectory if red
            // trajectory system does not account for setup from the other side

            // todo - maybe no longer flip?
            
            Trajectory.State driveState = 
                RedAllianceFlipUtility.apply(
                    mCustomTrajectoryGenerator.getDriveTrajectory().sample(currentTime)
                );
            RotationSequence.State holonomicRotationState = 
                RedAllianceFlipUtility.apply(
                    mCustomTrajectoryGenerator.getHolonomicRotationSequence().sample(currentTime)
                );
            

            /* 
            Trajectory.State driveState = 
       
                mCustomTrajectoryGenerator.getDriveTrajectory().sample(currentTime)
            ;
        RotationSequence.State holonomicRotationState = 
                mCustomTrajectoryGenerator.getHolonomicRotationSequence().sample(currentTime)
            ;
            */
                //Get Pose with vision: mRObotState.getPose()


            // Calculate velocity
            ChassisSpeeds nextDriveState =
                mCustomHolonomicDriveController.calculate(
                    mRobotState.getDriveOnlyPose(), driveState, holonomicRotationState);
            
            // Tell the Drive to Drive
            mDrive.setSwerveVelocity(nextDriveState);

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

    public double getEstimatedDuration()
    {
        return (null == mTrajectory) ? 0 : mTrajectory.getTotalTimeSeconds();
    }
}
