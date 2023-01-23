package frc.robot;

import java.util.Map;
import java.util.Optional;

import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Vision;
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

    // Reference to Photonvision
    photonVision mPhotonVision = photonVision.getInstance();

    // Robots Pose2d
    Pose2d mRobotPose;

    // Photon Vision Class to Estimate RobotPose based on last seen vision 
    RobotPoseEstimator mRobotPoseEstimator;

    // Robots Estimated Pose2d based 
    Pose2d mVisionBasedRobotPose;

    // Robots Vision Estimated Pose2d Latency
    double mVisionBasedRobotPoseLatencySeconds;

    // Valid Target 
    boolean mIsValidVisionPose;

    // Simulation or Real
    boolean mRealRobot;

    // Visulation Field
    Field2d mVisualField;

    private RobotState()
    {
        init();
    }

    private void init()
    {
       // Zero RobotState
       mRobotPose = new Pose2d(); 

       // Estimate the Robot's Pose on the Field
       //  Uses last seen AprilTag
       //  Various different strategies are available (AverageBestTargets, Closest_To_Camera_Height, Closest_To_Last_Pose,
       //                                              Closest_To_Reference_Pose, Lowest_Ambiguity)
       //  ref: https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
       mRobotPoseEstimator = new RobotPoseEstimator(FieldConstants.aprilTagField, 
                                                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonVision.CameraList
                                                    );

        // Robot Pose Calculated off of Vision
        mVisionBasedRobotPose = new Pose2d();
        mVisionBasedRobotPoseLatencySeconds = -1;
        mIsValidVisionPose = false;

        // Visulation Field
        mVisualField = new Field2d();
    }

    // Simulation or Real Robot
    public void setRealRobot(boolean value) {
        mRealRobot = value;
    }

    // Set Robot Pose
    public void setRobotPose(Pose2d pose) {
        mRobotPose = pose;
    }

    // Get Vision Estimated Robot Pose
    // Returns a Pair of the Pose2D of the Robot with a Latency Value
    public Pair<Pose2d, Double> getVisionEstimatedPose(Pose2d prevEstimatedRobotPose) {
        mRobotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = mRobotPoseEstimator.update();
        if (result.isPresent() && null != result.get().getFirst()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    // Get Vision Estimated Pose Latency (Seconds)
    public double getVisionEstimatedPoseLatency()
    {
        return mVisionBasedRobotPoseLatencySeconds;
    }

    // Get Vision Estimated Pose
    public Pose2d getVisionEstimatedPose()
    {
        return mVisionBasedRobotPose;
    }

    // Get if Vision Estimated Pose is Valid
    public boolean getVisionEstimatedPoseValid()
    {
        return mIsValidVisionPose;
    }

    // Update Robotstate
    // Called from Robot Periodic
    public void update()
    {
        // Update Pose2D based off of Vision
        Pair<Pose2d, Double> VisionBasedEstimate = getVisionEstimatedPose(mVisionBasedRobotPose);
        if(VisionBasedEstimate.getFirst() == null)
        {
            // No Pose2d Able to be found, zero the vision pose
            mVisionBasedRobotPose = new Pose2d();
            mVisionBasedRobotPoseLatencySeconds = -1;
        } else {
            // Valid Pose2D
            mVisionBasedRobotPose = VisionBasedEstimate.getFirst();
            mVisionBasedRobotPoseLatencySeconds = VisionBasedEstimate.getSecond();
        }

        // Evaluate if Latency is within Range
        if(mVisionBasedRobotPoseLatencySeconds != -1
            && Constants.Vision.kAllowedSecondsThreshold <= mVisionBasedRobotPoseLatencySeconds)
        {
            // Has a Vision Based Robot Pose and it is within acceptable latency
            mIsValidVisionPose = true;
        }else {
            // Either no Robot Pose or Outside of Latency
            mIsValidVisionPose = false;
        }

        // Simulation Only
        if(!mRealRobot)
        {
            // Update the Visualization Field
            updateSimVisualField();
        }
    }


    // TODO - Function to get Translation2D to a Node? 

    // Visual Plotting Field for Debug Perposes
    private void updateSimVisualField()
    {
        // X -> Field Length I believe.. 50 feet in field2d
        // Y -> Field Width
        /*
        mVisualField.setRobotPose(new Pose2d(FieldConstants.fieldLength/2, 
                                FieldConstants.fieldWidth/2, new Rotation2d()));
        */
        mVisualField.setRobotPose(mRobotPose);

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
            // Red Node
            Translation2d currTrans = FieldConstants.Grids.lowTranslations[i];
            Pose2d nodePose = new Pose2d(currTrans, new Rotation2d());
            FieldObject2d nodePoseObj = mVisualField.getObject("Red Node " + i);
            nodePoseObj.setPose(nodePose);

            // Blue Node
            Translation2d currTransBlue = FieldConstants.Grids.oppLowTranslations[i];
            Pose2d newPoseBlue = new Pose2d(currTransBlue, new Rotation2d());
            FieldObject2d nodePoseBlueObj = mVisualField.getObject("Blue Node " + i);
            nodePoseBlueObj.setPose(newPoseBlue);

            // Score Nodes
            

        }

        // Vision Estimated Robot Pose
        FieldObject2d simVisionPose = mVisualField.getObject("VisionEstimatedPose");
        simVisionPose.setPose(mVisionBasedRobotPose);

        // Vision Following Trajectory if applicable
        Trajectory driveTraj = AutoPilot.getInstance().getTrajectory();
        if(null != driveTraj)
        {
            mVisualField.getObject("Trajectory").setTrajectory(driveTraj);
        }

    }



    // Update Robot State Related Smart Dashboard
    public void updateSmartdashboard()
    {
        final String key = "RobotState/";
        SmartDashboard.putString(key + "Vision Pose", mVisionBasedRobotPose.toString());
        SmartDashboard.putNumber(key + "Vision Pose Latency", mVisionBasedRobotPoseLatencySeconds);
        SmartDashboard.putBoolean(key + "Vision Pose Valid", mIsValidVisionPose);
        SmartDashboard.putData(key + "Visual Field", mVisualField);
    }


}
