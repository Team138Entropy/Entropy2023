package frc.robot.util.simulation;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;

public class SimSwerveOdometry {
    private final SwerveDriveKinematics m_kinematics;
    private Pose2d m_poseMeters;
    private double m_prevTimeSeconds = -1;
  
    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
  
    /**
     * Constructs a SimSwerveOdometry object.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     * @param gyroAngle The angle reported by the gyroscope.
     * @param initialPose The starting position of the robot on the field.
     */
    public SimSwerveOdometry(
        SwerveDriveKinematics kinematics, Rotation2d gyroAngle, Pose2d initialPose) {
      m_kinematics = kinematics;
      m_poseMeters = initialPose;
      m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
      m_previousAngle = initialPose.getRotation();
      MathSharedStore.reportUsage(MathUsageId.kOdometry_SwerveDrive, 1);
    }
  
    /**
     * Constructs a SimSwerveOdometry object with the default pose at the origin.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     * @param gyroAngle The angle reported by the gyroscope.
     */
    public SimSwerveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle) {
      this(kinematics, gyroAngle, new Pose2d());
    }
  
    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param pose The position on the field that your robot is at.
     * @param gyroAngle The angle reported by the gyroscope.
     */
    public void resetPosition(Pose2d pose, Rotation2d gyroAngle) {
      m_poseMeters = pose;
      m_previousAngle = pose.getRotation();
      m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    }
  
    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public Pose2d getPoseMeters() {
      return m_poseMeters;
    }
  
    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose
     * over time. This method takes in the current time as a parameter to calculate period (difference
     * between two timestamps). The period is used to calculate the change in distance from a
     * velocity. This also takes in an angle parameter which is used instead of the angular rate that
     * is calculated from forward kinematics.
     *
     * @param currentTimeSeconds The current time in seconds.
     * @param gyroAngle The angle reported by the gyroscope.
     * @param moduleStates The current state of all swerve modules. Please provide the states in the
     *     same order in which you instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d updateWithTime(
        double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
      double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
      m_prevTimeSeconds = currentTimeSeconds;
  
      var angle = gyroAngle.plus(m_gyroOffset);
  
      var chassisState = m_kinematics.toChassisSpeeds(moduleStates);
      var newPose =
          m_poseMeters.exp(
              new Twist2d(
                  chassisState.vxMetersPerSecond * period,
                  chassisState.vyMetersPerSecond * period,
                  angle.minus(m_previousAngle).getRadians()));
  
      m_previousAngle = angle;
      m_poseMeters = new Pose2d(newPose.getTranslation(), angle);
  
      return m_poseMeters;
    }
  
    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose
     * over time. This method automatically calculates the current time to calculate period
     * (difference between two timestamps). The period is used to calculate the change in distance
     * from a velocity. This also takes in an angle parameter which is used instead of the angular
     * rate that is calculated from forward kinematics.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param moduleStates The current state of all swerve modules. Please provide the states in the
     *     same order in which you instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
      return updateWithTime(WPIUtilJNI.now() * 1.0e-6, gyroAngle, moduleStates);
    }
}
