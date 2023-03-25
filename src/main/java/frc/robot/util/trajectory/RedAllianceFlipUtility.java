package frc.robot.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

//todo - hate this import
import frc.robot.FieldConstants;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */

public class RedAllianceFlipUtility {
    /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          FieldConstants.fieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   * This function has been modified from the original one, and no longer inverts the X axis
   * Because this system is field relative, the trajectory will always want to go forward
   */
  public static Trajectory.State apply(Trajectory.State state) {
    if (shouldFlip()) {
      return new Trajectory.State(
          state.timeSeconds,
          state.velocityMetersPerSecond,
          state.accelerationMetersPerSecondSq,
          new Pose2d(
              state.poseMeters.getX(),
              FieldConstants.fieldWidth - state.poseMeters.getY(),
              new Rotation2d(
                  state.poseMeters.getRotation().getCos(),
                  -state.poseMeters.getRotation().getSin())),
          -state.curvatureRadPerMeter);
    } else {
      return state;
    }
  }

  /** Flips a rotation sequence state based on the current alliance color. */
  // Modified Function, invert Sin only
  public static RotationSequence.State apply(RotationSequence.State state) {
    if (shouldFlip()) {
      return new RotationSequence.State(
          new Rotation2d(state.position.getCos(), -state.position.getSin()),
          -state.velocityRadiansPerSec);
    } else {
      return state;
    }
  }

  private static boolean shouldFlip() {
    return DriverStation.getAlliance() == Alliance.Red;
  }
}
