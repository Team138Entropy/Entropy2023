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
import frc.robot.util.trajectory.PoseType;
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

    // .02 M/S 2
    private double mMaxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);
    private Timer mTimer;

    private final PIDController mXController = new PIDController(2.5, 0.0, 0.0);
    private final PIDController mYController = new PIDController(2.5, 0.0, 0.0);
    private final PIDController mThetaController = new PIDController(
        4, 0.0, 0.0);

    // Trajectory Genrator, Drive Controller
    private final CustomTrajectoryGenerator mCustomTrajectoryGenerator = new CustomTrajectoryGenerator();
    private final CustomHolonomicDriveController mCustomHolonomicDriveController = new CustomHolonomicDriveController(
        mXController, mYController, mThetaController
    );

    private List<Pair<Pose2d, PoseType>> mPoses;
    private Trajectory mTrajectory = null;
    private Rotation2d mOrentation;
    private boolean mUseVision = false;
    private double mEndVelocity = 1.4;


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

    // Adds a Holonomic Pose
    public void addPose(Pose2d pose)
    {
        mPoses.add(new Pair<Pose2d, PoseType>(pose, PoseType.Holonomic));
    }

    // Adds a Holonomic Pose
    public void addHolonomicPose(Pose2d pose)
    {
        mPoses.add(new Pair<Pose2d, PoseType>(pose, PoseType.Holonomic));
    }

    // Adds a Differential Pose
    public void addDifferentialPose(Pose2d pose)
    {
        mPoses.add(new Pair<Pose2d, PoseType>(pose, PoseType.Differential));
    }

    // Adds a Translation Only
    public void addTranslation(Translation2d trans)
    {
        mPoses.add(new Pair<Pose2d, PoseType>(
            new Pose2d(
                trans, new Rotation2d()
            ),
            PoseType.TranslationOnly
        ));
    }
    
    public void addWaypoint(Waypoint wp)
    {
        Pose2d posePoint = new Pose2d(
            wp.getPose(mRobotState.getAlliance()).getTranslation(),
            wp.getPose(mRobotState.getAlliance()).getRotation()
        );
        mPoses.add(new Pair<Pose2d, PoseType>(posePoint, PoseType.Holonomic));
    }

    public void addWaypoint(Waypoint wp, SwerveRotation rotation)
    {
        Pose2d posePoint = new Pose2d(
            wp.getPose(mRobotState.getAlliance()).getTranslation(),
            rotation.getRotation()
        );
        mPoses.add(new Pair<Pose2d, PoseType>(posePoint, PoseType.Holonomic));
    }

    // Tell the Custom Holonomic Controller to use Vision
    public void useVision() {
        mUseVision = true;
    }
    
    // Generate the Trajectory
    public void generate()
    {
        TrajectoryConfig config = new TrajectoryConfig( //Vel, Accel
            mMaxVelocityMS, mMaxVAccelerationMSS)
            .setKinematics(mDrive.getSwerveKinematics())
            .setStartVelocity(0)
            .setEndVelocity(mEndVelocity).addConstraint(
                new CentripetalAccelerationConstraint(mMaxCentripetalAccelerationMetersPerSec2)
        );

        // Create Generator and Generate
        mCustomTrajectoryGenerator.generateWithPoses(config, mPoses);

        // Store Trajectory
        mTrajectory = mCustomTrajectoryGenerator.getDriveTrajectory();
    }

    public void setEndVelocity(double vel){
        mEndVelocity = vel;
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
            
            /*
            Trajectory.State driveState = 
                    mCustomTrajectoryGenerator.getDriveTrajectory().sample(currentTime);
            RotationSequence.State holonomicRotationState = 
                    mCustomTrajectoryGenerator.getHolonomicRotationSequence().sample(currentTime);
            */
           
           // Flip if Red
           // X Side will behave the same, Y will be different
           Trajectory.State driveState = 
                RedAllianceFlipUtility.apply(
                    mCustomTrajectoryGenerator.getDriveTrajectory().sample(currentTime)
                );
            RotationSequence.State holonomicRotationState = 
                RedAllianceFlipUtility.apply(
                    mCustomTrajectoryGenerator.getHolonomicRotationSequence().sample(currentTime)
            );
        

            // Get Robot Pose to Calculate Error
            // Determine if using Vision Based Pose or Drive Only Pose
            Pose2d currentRobotPose = mRobotState.isRealRobot() ? 
                (mUseVision ? mRobotState.getPose() : mRobotState.getDriveOnlyPose()) :
                (mRobotState.getDriveOnlySimPose());

            // Calculate velocity
            ChassisSpeeds nextDriveState =
                mCustomHolonomicDriveController.calculate(
                    currentRobotPose, driveState, holonomicRotationState);
            
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
