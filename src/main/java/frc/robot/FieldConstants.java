package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. Important to rember the fields are mirrored and are not
 * symetric
 * TODO - how do we plot this?
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public final class FieldConstants {
    // Field Dimensions (Meters)
    public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
    public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);

    // Width of Tape used throughout the field
    public static final double tapeWidth = Units.inchesToMeters(2.0);

    // AprilTag IDs & Locations (do not flip for red alliance)
    public static final List<AprilTag> aprilTags = List.of(
      new AprilTag(1, new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI))
      ),
      new AprilTag(2, new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(108.19),
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI))
      ),
      new AprilTag(3, new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI))
      ),
      new AprilTag(4, new Pose3d(
          Units.inchesToMeters(636.96),
          Units.inchesToMeters(265.74),
          Units.inchesToMeters(27.38),
          new Rotation3d(0.0, 0.0, Math.PI))
      ),
      new AprilTag(5, new Pose3d(
          Units.inchesToMeters(14.25),
          Units.inchesToMeters(265.74),
          Units.inchesToMeters(27.38),
          new Rotation3d())
      ),
      new AprilTag(6, new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
          Units.inchesToMeters(18.22),
          new Rotation3d())
      ), 
      new AprilTag(7, new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(108.19),
          Units.inchesToMeters(18.22),
          new Rotation3d())
      ),
      new AprilTag(8, new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(42.19),
          Units.inchesToMeters(18.22),
          new Rotation3d())
      )
    );

    // Get an April Tag by Id
    public static AprilTag getAprilTag(int tagID)
    {
        AprilTag result = null;
        for(int i = 0; i < aprilTags.size(); ++i)
        {
            if(aprilTags.get(i).ID == tagID)
            {
                result = aprilTags.get(i);
                break;
            }
        }
        return result;
    }

    // April Tag Field Locations
    public static final AprilTagFieldLayout aprilTagField = new AprilTagFieldLayout(aprilTags, fieldLength, fieldWidth);

  // Dimensions for Community 
  public static final class Community {
        // Region dimensions
        public static final double innerX = 0.0;
        public static final double midX =
            Units.inchesToMeters(132.375); // Tape to the left of charging station
        public static final double outerX =
            Units.inchesToMeters(193.25); // Tape to the right of charging station
        public static final double leftY = Units.feetToMeters(18.0);
        public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
        public static final double rightY = 0.0;
        public static final Translation2d[] regionCorners =
            new Translation2d[] {
              new Translation2d(innerX, rightY),
              new Translation2d(innerX, leftY),
              new Translation2d(midX, leftY),
              new Translation2d(midX, midY),
              new Translation2d(outerX, midY),
              new Translation2d(outerX, rightY),
            };

        // Charging station dimensions
        public static final double chargingStationLength = Units.inchesToMeters(76.125);
        public static final double chargingStationWidth = Units.inchesToMeters(97.25);
        public static final double chargingStationOuterX = outerX - tapeWidth;
        public static final double chargingStationInnerX =
            chargingStationOuterX - chargingStationLength;
        public static final double chargingStationLeftY = midY - tapeWidth;
        public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
        public static final Translation2d[] chargingStationCorners =
            new Translation2d[] {
              new Translation2d(chargingStationInnerX, chargingStationRightY),
              new Translation2d(chargingStationInnerX, chargingStationLeftY),
              new Translation2d(chargingStationOuterX, chargingStationRightY),
              new Translation2d(chargingStationOuterX, chargingStationLeftY)
            };
    
  }

  // Dimensions for grids and nodes
  public static final class Grids {
    // X layout
    public static final double outerX = Units.inchesToMeters(54.25);
    public static final double lowX =
        outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
    public static final double midX = outerX - Units.inchesToMeters(22.75);
    public static final double highX = outerX - Units.inchesToMeters(39.75);

    // Y layout
    public static final int nodeRowCount = 9;
    public static final double nodeFirstY = Units.inchesToMeters(20.19);
    public static final double nodeSeparationY = Units.inchesToMeters(22.0);

    // Z layout
    public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
    public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
    public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
    public static final double highConeZ = Units.inchesToMeters(46.0);
    public static final double midConeZ = Units.inchesToMeters(34.0);

    // Translations (all nodes in the same column/row have the same X/Y coordinate)
    public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
    public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
    public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
    public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
    public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

    // Blue Side Low Translation
    public static final Translation2d[] oppLowTranslations = new Translation2d[nodeRowCount];

    // Artifical Scoring Targets -> Target the Initial Scoring Position and then adjust to final scoring position
    // Initial Scoring Positions
    public static final Translation2d[] redInitScorePosition = new Translation2d[nodeRowCount];
    public static final Translation2d[] blueInitScorePosition = new Translation2d[nodeRowCount];
    public static final double scorePosOffset = 0.15;

    // Final Scoring Positions
    public static final Translation2d[] redFinalScorePosition = new Translation2d[nodeRowCount];
    public static final Translation2d[] blueFinalScorePosition = new Translation2d[nodeRowCount];
    public static final double scoreFinalPosOffset = .02;

    public static final Translation2d[] redFinalScorePositionFlipped = new Translation2d[nodeRowCount];
    public static final Translation2d[] blueFinalScorePositionFlipped = new Translation2d[nodeRowCount];
    static {
      for (int i = 0; i < nodeRowCount; i++) {
        boolean isCube = i == 1 || i == 4 || i == 7;
        lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
        midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
        mid3dTranslations[i] =
            new Translation3d(midX, nodeFirstY + nodeSeparationY * i, isCube ? midCubeZ : midConeZ);
        high3dTranslations[i] =
            new Translation3d(
                highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
        highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);

        // create opposite end low translation (blue nodes)
        oppLowTranslations[i] = new Translation2d(fieldLength - lowX, nodeFirstY + nodeSeparationY * i);

        // Arificial Scoring Positions
        // Initial Scoring Positions
        redInitScorePosition[i] = oppLowTranslations[i].plus(new Translation2d(scorePosOffset, 0));
        blueInitScorePosition[i] = lowTranslations[i].minus(new Translation2d(scorePosOffset, 0));

        // Final Scoring Positions
        redFinalScorePosition[i] = redInitScorePosition[i].minus(new Translation2d(scoreFinalPosOffset, 0));
        blueFinalScorePosition[i] = blueInitScorePosition[i].plus(new Translation2d(scoreFinalPosOffset, 0));
      }
      
      // Use to Flip the Order to get in Grid 1 -> 9
      for (int i = 0; i < nodeRowCount; i++) {
        redFinalScorePositionFlipped[i] = redFinalScorePosition[nodeRowCount - i - 1];
        blueFinalScorePositionFlipped[i] = blueFinalScorePosition[i]; // blue is already flipped
      }
    }

    // Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
    public static final double complexLowXCones =
        outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone nodes
    public static final double complexLowXCubes = lowX; // Centered X under cube nodes
    public static final double complexLowOuterYOffset =
        nodeFirstY - Units.inchesToMeters(3.0) - (Units.inchesToMeters(25.75) / 2.0);

    public static final Translation2d[] complexLowTranslations =
        new Translation2d[] {
          new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
          new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
          new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
          new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
          new Translation2d(
              complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),
        };
  }


}
