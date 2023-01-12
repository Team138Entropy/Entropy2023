package frc.robot;

import java.util.Map;
import java.util.Optional;

import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Vision;
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

    // Photon Vision Class to Estimate RobotPose based on last seen vision 
    RobotPoseEstimator mRobotPoseEstimator;

    // Robots Estimated Pose2d based 
    Pose2d mVisionBasedRobotPose;

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
       mRobotPoseEstimator = new RobotPoseEstimator(FieldConstants.aprilTagField, 
                                                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonVision.CameraList
                                                    );

        // Robot Pose Calculated off of Vision
        mVisionBasedRobotPose = new Pose2d();
    }

    // Get Vision Estimated Robot Pose
    // Returns a Pair of the Pose2D of the Robot with a Latency Value
    public Pair<Pose2d, Double> getVisionEstimatedPose(Pose2d prevEstimatedRobotPose) {
        mRobotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = mRobotPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
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
        } else {
            // Valid Pose2D
            mVisionBasedRobotPose = VisionBasedEstimate.getFirst();
        }
    }


    // TODO - Function to get Translation2D to a Node? 



    // Update Robot State Related Smart Dashboard
    public void updateSmartdashboard()
    {
        final String key = "RobotState/";
        SmartDashboard.putString(key + "Vision Pose", mVisionBasedRobotPose.toString());
    }


}
