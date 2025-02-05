package frc.robot.util.trajectory;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

/** A trajectory waypoint, including a translation and optional drive/holonomic rotations. */
public class TrajectoryWaypoint {
  private final Translation2d translation;
  private final Rotation2d driveRotation;
  private final Rotation2d holonomicRotation;

  /** Constructs a Waypoint at the origin and without a drive or holonomic rotation. */
  public TrajectoryWaypoint() {
    this(new Translation2d());
  }

  /**
   * Constructs a Waypoint with a translation, drive rotation, and holonomic rotation.
   *
   * @param translation Waypoint position (required)
   * @param driveRotation Drive velocity rotation (optional, can be null)
   * @param holonomicRotation Holonomic rotation (optional, can be null)
   */
  public TrajectoryWaypoint(
      Translation2d translation, Rotation2d driveRotation, Rotation2d holonomicRotation) {
    this.translation = requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = driveRotation;
    this.holonomicRotation = holonomicRotation;
  }

  /**
   * Constructs a Waypoint with a translation (but no drive or holonomic rotation).
   *
   * @param translation Waypoint position (required)
   */
  public TrajectoryWaypoint(Translation2d translation) {
    this.translation = requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = null;
    this.holonomicRotation = null;
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the drive rotation)
   */
  public static TrajectoryWaypoint fromDifferentialPose(Pose2d pose) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new TrajectoryWaypoint(pose.getTranslation(), pose.getRotation(), null);
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the drive rotation)
   * @param holonomicRotation Holonomic rotation
   */
  public static TrajectoryWaypoint fromDifferentialPose(Pose2d pose, Rotation2d holonomicRotation) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new TrajectoryWaypoint(pose.getTranslation(), pose.getRotation(), holonomicRotation);
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the holonomic rotation)
   */
  public static TrajectoryWaypoint fromHolonomicPose(Pose2d pose) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new TrajectoryWaypoint(pose.getTranslation(), null, pose.getRotation());
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the holonomic rotation)
   * @param driveRotation Drive rotation
   */
  public static TrajectoryWaypoint fromHolonomicPose(Pose2d pose, Rotation2d driveRotation) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new TrajectoryWaypoint(pose.getTranslation(), driveRotation, pose.getRotation());
  }

  /** Returns the translation component of the waypoint. */
  public Translation2d getTranslation() {
    return translation;
  }

  /**
   * Returns the drive rotation component of the waypoint (or an empty optional if not specified).
   */
  public Optional<Rotation2d> getDriveRotation() {
    return Optional.ofNullable(driveRotation);
  }

  /**
   * Returns the holonomic rotation component of the waypoint (or an empty optional if not
   * specified).
   */
  public Optional<Rotation2d> getHolonomicRotation() {
    return Optional.ofNullable(holonomicRotation);
  }
}
