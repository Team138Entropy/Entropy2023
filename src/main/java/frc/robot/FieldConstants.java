package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

    // Scoring Location (Each Grid Consists of 9 Nodes)
    public static final class Node {
        // Enum for Node Type
        public Enums.GamePiece mNodeType;

        // Nodes 3D Pose
        final Pose3d mNodeLocation = new Pose3d(0, 0, 0, new Rotation3d());

        public Node()
        {

        }
    }

    // 3 Grids on each Side of the Field, Each Made up of 9 Nodes
    public static final class Grid {
        public final Node mNodes[];
        private final int nodeCount;

        public Grid()
        {
            nodeCount = 9;
            mNodes = new Node[9];
            for(int i = 0; i < 9; ++i)
            {
                mNodes[i] = new Node();
            }
        }
    }

    // TODO: Enum of Grid Locations
    public static final Grid mGrids[] = {
        new Grid(),
        new Grid(),
        new Grid(),
        new Grid(), 
        new Grid(), 
        new Grid(),
    };


}
