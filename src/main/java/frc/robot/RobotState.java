package frc.robot;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Vision;
import frc.robot.Enums.CurrentMode;
import frc.robot.subsystems.Drive;
import frc.robot.util.drivers.Pigeon;
import frc.robot.vision.AutoPilot;
import frc.robot.vision.photonVision;

/**
*  RobotState
*  Contains functionality related to the Robot and its position on the field.
*/
public class RobotState {
    private static RobotState mInstance;

    public static synchronized RobotState getInstance() {
        if (mInstance == null) {
          mInstance = new RobotState();
        }
        return mInstance;
    }

    // Pigeon
    private final Pigeon mPigeon = Pigeon.getInstance();

    // Drive
    private final Drive mDrive = Drive.getInstance();

    // Reference to Photonvision
    private final photonVision mPhotonVision = photonVision.getInstance();

    // Vision Pose Estimators 
    enum PoseCameras {
        FrontCamera, 
        BackCamera,
        Count
    };
    private final int mPhotonPoseEstimatorCount = PoseCameras.Count.ordinal();
    private PhotonPoseEstimator[] mPhotonPoseEstimators;
    private Pose2d[] mVisionEstimatedPoses;
    private boolean[] mVisionEstimatedPosesValid;
    private double[] mVisionEstimatedPoseTimestamp;

    boolean mHasInitialVision = false;

    // Overall Drive Pose Estimator
    /*
    * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
    * while still accounting for measurement noise.
    *
    * <p>This method can be called as infrequently as you want, as long as you are calling {@link
    * SwerveDrivePoseEstimator#update} every loop.
    *
    * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
    * recommend only adding vision measurements that are already within one meter or so of the
    * current pose estimate.
    */
    SwerveDrivePoseEstimator mSwerveDrivePoseEstimator;

    // Drive Only Pose Estimator
    // Pose that does not take Vision into account and only uses Odometry
    SwerveDrivePoseEstimator mSwerveDriveOnlyPoseEstimator;

    // Threshold of Vision Targets that will recieve 
    private final double mVisionDistanceThreshold = 12;

    // Pose Estimate Based Pose (Using Vision + Drive)
    Pose2d mRobotPose;

    // Pose Estimated based on Drive only
    Pose2d mRobotPoseDriveOnly;

    // Target Pose
    Pose2d mTargetPose;

    // Simulation or Real
    private boolean mRealRobot = true;

    // Alliance Color
    private Alliance mAlliance;

    // Current Robot Mode
    private CurrentMode mCurrentMode;

    // Visulation Field
    private Field2d mVisualField;

    private RobotState()
    {
        init();
    }

    private void init()
    {
       // Estimate the Robot's Pose on the Field
       //  Uses last seen AprilTag
       //  Various different strategies are available (AverageBestTargets, Closest_To_Camera_Height, Closest_To_Last_Pose,
       //                                              Closest_To_Reference_Pose, Lowest_Ambiguity)
       //  ref: https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
       // https://en.wikipedia.org/wiki/Perspective-n-Point?fbclid=IwAR17ci8IfdLgKKkdBSUazSIzoDU9U2vXsbaae1qVZvg4o5O0mpnjKTY5KMQ
       // PnP seems to be the best startegy - use all visble tags to compute single best
       // https://www.chiefdelphi.com/t/photonvision-2023-official-release/421061/70?fbclid=IwAR0O3sD70Zu8DkXqv3nPMKKDqF2eyeo_wpO0Hgh_pa3g4OH6Ft-lEyQSNtc
       PoseStrategy chosenStrategy = PoseStrategy.MULTI_TAG_PNP;
       // Initialize Each Pose Estimator [FrontCamera, BackCamera]
       mPhotonPoseEstimators = new PhotonPoseEstimator[mPhotonPoseEstimatorCount];
       mVisionEstimatedPoses = new Pose2d[mPhotonPoseEstimatorCount];
       mVisionEstimatedPosesValid = new boolean[mPhotonPoseEstimatorCount];
       mVisionEstimatedPoseTimestamp = new double[mPhotonPoseEstimatorCount];
       for(int i = 0; i < mPhotonPoseEstimatorCount; ++i)
       {
            // Create Pose Estimator
            mPhotonPoseEstimators[i] = new PhotonPoseEstimator(
                FieldConstants.aprilTagField, 
                chosenStrategy, 
                photonVision.CameraList.get(i).getFirst(), //PhotonCamera
                photonVision.CameraList.get(i).getSecond() //Transform3d
            );

            // Zero Estimated Poses and if the Poses are valid
            mVisionEstimatedPoses[i] = new Pose2d();
            mVisionEstimatedPosesValid[i] = false;
            mVisionEstimatedPoseTimestamp[i] = 0;
       }

        // Swerve Drive Pose Estimator
        // Using Default Std Deviations (Drive .1, Vision .9)
        mSwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(mDrive.getSwerveKinematics(), 
            mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : mPigeon.getSimYaw().getWPIRotation2d(), 
            mRealRobot ? mDrive.getModulePositions() : mDrive.getSimSwerveModulePositions(),
            new Pose2d()
        );
        
        // Swerive Drive Only Pose Estimator
        mSwerveDriveOnlyPoseEstimator = new SwerveDrivePoseEstimator(mDrive.getSwerveKinematics(), 
            mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : mPigeon.getSimYaw().getWPIRotation2d(), 
            mRealRobot ? mDrive.getModulePositions() : mDrive.getSimSwerveModulePositions(),
            new Pose2d()
        );

        // Default Pose Estimations
        mRobotPose = new Pose2d();
        mRobotPoseDriveOnly = new Pose2d();

        // Target Pose 
        mTargetPose = new Pose2d();

        // Default to Blue Alliance 
        mAlliance = Alliance.Blue;

        // Visulation Field
        mVisualField = new Field2d();
    }

    // Simulation or Real Robot
    public void setRealRobot(boolean value) {
        mRealRobot = value;

        // Change Pose Stategy if sim robot
        if(!mRealRobot)
        {
            for(int i = 0; i < mPhotonPoseEstimatorCount; ++i) 
            {
                mPhotonPoseEstimators[i].setPrimaryStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            }
        }
    }



    // Resets Odometry and Robot State
    public void resetPosition(Pose2d pose)
    {
        // Simulation 
        // Since the Robot will have been physically moved, need to update in sim
        if(!mRealRobot)
        {
            // this is no good because it affects the adjust angle
            mPigeon.setSimYaw((pose.getRotation().getDegrees()));
        }

        // Move Odometry
        mDrive.resetOdometry(pose);

        // Reset Pose Estimators
        // Drive Pose Estimator with Vision
        mSwerveDrivePoseEstimator.resetPosition(
            mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : mPigeon.getSimYaw().getWPIRotation2d(), 
            mRealRobot ? mDrive.getModulePositions() : mDrive.getSimSwerveModulePositions(),
            pose
        );

        // Drive Pose Estimator without Vision
        mSwerveDriveOnlyPoseEstimator.resetPosition(
            mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : mPigeon.getSimYaw().getWPIRotation2d(), 
            mRealRobot ? mDrive.getModulePositions() : mDrive.getSimSwerveModulePositions(),
            pose
        );
    }

    // Get Vision Estimated Robot Pose
    // Returns a Pair of the Pose2D of the Robot with a Latency Value
    // Sets a Reference pose for some algorithms that use the pose
    public Pair<Pose2d, Double> getVisionEstimatedPose(int poseEstimatorIndex, Pose2d referencePose) {
        // Set Reference Pose (don't think this is used for chosen stategy)
        mPhotonPoseEstimators[poseEstimatorIndex].setReferencePose(referencePose);

        // Timestamp back from photonVision should be in terms of fpga timestamp     
        Optional<EstimatedRobotPose> result = mPhotonPoseEstimators[poseEstimatorIndex].update();
        if (result.isPresent() && null != result.get().estimatedPose) {
            return new Pair<Pose2d, Double>(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    // Get Vision Estimated Pose Latency (Seconds)

    // Update Robotstate
    // Called from Robot Periodic
    public void update()
    {
        // Update with Robot Odometry
        // Drive Pose Estimator with Vision
        mSwerveDrivePoseEstimator.update(
            mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : mPigeon.getSimYaw().getWPIRotation2d(), 
            mRealRobot ? mDrive.getModulePositions() : mDrive.getSimSwerveModulePositions()
        );

        // Drive Pose Estimator without Vision
        mSwerveDriveOnlyPoseEstimator.update(
            mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : mPigeon.getSimYaw().getWPIRotation2d(), 
            mRealRobot ? mDrive.getModulePositions() : mDrive.getSimSwerveModulePositions()
        );

        // OVerall Swerve Drive Estimated Position
        // This is the current estimate before vision updates
        Pose2d currentSwervePose = mSwerveDrivePoseEstimator.getEstimatedPosition();

        // Iterate each PhotonVisionPose Estimator and 
        // Initialize Each Pose Estimator [FrontCamera, BackCamera]
        for(int i = 0; i < mPhotonPoseEstimatorCount; ++i)
        {
            // Get the Latest Pose with Latency Compensation
            // Pass in a reference pose in case the algorithm calls for it
            Pair<Pose2d, Double> VisionBasedEstimate = getVisionEstimatedPose(i, currentSwervePose);

            // If Valid Vision Pose Estimate is Returned
            if(null != VisionBasedEstimate.getFirst())
            {
                // Pose Estimator Reconmends not adding a pose more then a meter away
                // for now just add it
                boolean addPose = true;

                // Lower the Standard Deviation the more the vision pose is trusted
                // Distance To Target in Meters
                double distanceToTarget = Math.abs(VisionBasedEstimate.getFirst().getTranslation().getDistance(
                    currentSwervePose.getTranslation()
                ));

                // Vision Estimate Std Deviation
                double stdDeviationCoefficient = .75;

                // If within Threshold
                if(distanceToTarget <= mVisionDistanceThreshold)
                {
                    stdDeviationCoefficient = .3;
                }

                // Add Vision Pose 
                if(addPose)
                {
                    mSwerveDrivePoseEstimator.addVisionMeasurement(
                        VisionBasedEstimate.getFirst(), // Pose2d
                        VisionBasedEstimate.getSecond(), // Timestamp Seconds
                        VecBuilder.fill(stdDeviationCoefficient, stdDeviationCoefficient, stdDeviationCoefficient)
                    );

                    // Set Valid and Store Pose
                    mVisionEstimatedPoses[i] = VisionBasedEstimate.getFirst();
                    mVisionEstimatedPosesValid[i] = true;
                    mVisionEstimatedPoseTimestamp[i] = VisionBasedEstimate.getSecond();
                }
            } else {
                // No Valid Vision Pose
                mVisionEstimatedPoses[i] = new Pose2d();
                mVisionEstimatedPosesValid[i] = false;
                mVisionEstimatedPoseTimestamp[i] = 0;
            }
        }

        // Get Overall System Pose Estimate
        // Accounts for Swerve System Odometry and Vision Poses
        mRobotPose = mSwerveDrivePoseEstimator.getEstimatedPosition();

        // Get the Drive Pose Estimate
        mRobotPoseDriveOnly = mSwerveDriveOnlyPoseEstimator.getEstimatedPosition();

        // Simulation Only
        if(!mRealRobot)
        {
            // Update the Visualization Field
            updateSimVisualField();
        }
    }

    // Gets Overall Pose of the Robot
    //  This Pose uses the Vision System for TranslationXY
    //     and the pidgeon for rotation
    public Pose2d getPose()
    {
        return new Pose2d(
           mRobotPose.getTranslation(),
           mRealRobot ? mPigeon.getYaw().getWPIRotation2d() : mPigeon.getSimYaw().getWPIRotation2d()
        );
    }

    // Gets Drive Only Robot Pose
    public Pose2d getDriveOnlyPose()
    {
        return mRobotPoseDriveOnly;
    }

    // Visual Plotting Field for Debug Perposes
    private void updateSimVisualField()
    {
        // X -> Field Length I believe.. 50 feet in field2d
        // Y -> Field Width
        /*
        mVisualField.setRobotPose(new Pose2d(FieldConstants.fieldLength/2, 
                                FieldConstants.fieldWidth/2, new Rotation2d()));
        */
        mVisualField.setRobotPose(getPose());

        // Iterate April Tags and Plot
        for(int i = 0; i < FieldConstants.aprilTags.size(); i++)
        {
            AprilTag currTag = FieldConstants.aprilTags.get(i);
            String currName = "AprilTag " + currTag.ID;
            Pose3d currPose3d = currTag.pose;
            Pose2d currPose2d = currPose3d.toPose2d();
            FieldObject2d fieldObj = mVisualField.getObject(currName);
            fieldObj.setPose(currPose2d);
        }

        // Iterate Node Scoring Positions (In Front of the Lowest Node)
        for(int i = 0; i < FieldConstants.Grids.lowTranslations.length; i++)
        {
            // Blue Node
            Translation2d currTrans = FieldConstants.Grids.lowTranslations[i];
            Pose2d nodePose = new Pose2d(currTrans, new Rotation2d());
            FieldObject2d nodePoseObj = mVisualField.getObject("Blue Node " + i);
            nodePoseObj.setPose(nodePose);

            // Red Node
            Translation2d currTransRed = FieldConstants.Grids.oppLowTranslations[i];
            Pose2d newPoseRed = new Pose2d(currTransRed, new Rotation2d());
            FieldObject2d nodePoseRedObj = mVisualField.getObject("Red Node " + i);
            nodePoseRedObj.setPose(newPoseRed);

            // Score Nodes
                   

        }

        // Staging Locations
        for(int i = 0; i < FieldConstants.StagingLocations.blueStagingLocations.length; ++i)
        {
            Translation2d stageLocationBlue = FieldConstants.StagingLocations.blueStagingLocations[i];
            Translation2d stageLocationRed = FieldConstants.StagingLocations.redStagingLocations[i];
            Pose2d stagePoseBlue = new Pose2d(stageLocationBlue, new Rotation2d());
            Pose2d stagePoseRed = new Pose2d(stageLocationRed, new Rotation2d());

            FieldObject2d RedStagePoseObj = mVisualField.getObject("R Stage " + i);
            FieldObject2d BlueStagePoseObj = mVisualField.getObject("B Stage " + i);
            RedStagePoseObj.setPose(stagePoseRed);
            BlueStagePoseObj.setPose(stagePoseBlue);
        }

        // Charging Station Corners
        for(int i = 0; i < FieldConstants.Community.chargingStationCornersBlue.length; ++i)
        {
            Translation2d cornerB = FieldConstants.Community.chargingStationCornersBlue[i];
            Pose2d cornerPose = new Pose2d(cornerB, new Rotation2d());
            FieldObject2d cornerPoseObj = mVisualField.getObject("B CS Corner " + i);
            cornerPoseObj.setPose(cornerPose);

            Translation2d cornerR = FieldConstants.Community.chargingStationCornersRed[i];
            Pose2d cornerPoseRed = new Pose2d(cornerR, new Rotation2d());
            FieldObject2d RedCornerPoseObj = mVisualField.getObject("R CS Corner " + i);
            RedCornerPoseObj.setPose(cornerPoseRed);
        }
        
        // Vision Following Trajectory if applicable
        /*
        Trajectory driveTraj = AutoPilot.getInstance().getTrajectory();
        if(null != driveTraj)
        {
            mVisualField.getObject("Trajectory").setTrajectory(driveTraj);
        }
        */

        // Vision Poses
        for(int i = 0; false && i < mPhotonPoseEstimatorCount; ++i) 
        {
            PoseCameras currentCamera = PoseCameras.values()[i];
            final String cameraPoseKey = currentCamera.toString() + "_VisionPose";
            FieldObject2d visionPoseObject = mVisualField.getObject(cameraPoseKey);
            visionPoseObject.setPose(mVisionEstimatedPoses[i]);
        }
       
        // Desired Pose
        FieldObject2d targetPose2d = mVisualField.getObject("TargetedPose");
    }

    public void setTargetPose(Pose2d pose)
    {
        mTargetPose = pose;
    }

    // Set Current Alliance
    public boolean setAlliance(Alliance allianceC)
    {
        boolean different = false;
        if(mAlliance != allianceC) {
            different = true;
        }
        mAlliance = allianceC;
        return different;
    }

    // Get Current Alliance
    public Alliance getAlliance()
    {
        return mAlliance;
    }

   

    // Update Robot State Related Smart Dashboard
    public void updateSmartdashboard()
    {
        final String key = "RobotState/";
        SmartDashboard.putData(key + "Visual Field", mVisualField);
        SmartDashboard.putString(key + "Robot Pose", mRobotPose.toString());
        SmartDashboard.putString(key + "Robot Pose Drive Only", mRobotPoseDriveOnly.toString());
        SmartDashboard.putString(key + "Target Pose", mTargetPose.toString());

        // Each Individual Vision Pose
        final String visionPoses = key + "Vision Poses/";
        for(int i = 0; i < mPhotonPoseEstimatorCount; ++i) 
        {
            PoseCameras currentCamera = PoseCameras.values()[i];
            final String cameraPoseKey = visionPoses + currentCamera.toString() + "_VisionPose";
            SmartDashboard.putString(cameraPoseKey, mVisionEstimatedPoses[i].toString());
        }
    }
}
