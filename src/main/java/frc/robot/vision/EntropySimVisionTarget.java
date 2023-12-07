package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class EntropySimVisionTarget {
    Pose3d targetPose;
    double targetWidthMeters;
    double targetHeightMeters;
    double tgtAreaMeters2;
    int targetID;

    /**
     * Describes a vision target located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param targetPos Pose3d of the target in field-relative coordinates
     * @param targetWidthMeters Width of the outer bounding box of the target in meters.
     * @param targetHeightMeters Pair Height of the outer bounding box of the target in meters.
     */
    public EntropySimVisionTarget(
            Pose3d targetPos, double targetWidthMeters, double targetHeightMeters, int targetID) {
        this.targetPose = targetPos;
        this.targetWidthMeters = targetWidthMeters;
        this.targetHeightMeters = targetHeightMeters;
        this.tgtAreaMeters2 = targetWidthMeters * targetHeightMeters;
        this.targetID = targetID;
    }
}

