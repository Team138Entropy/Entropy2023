package frc.robot.util.drivers;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;
import frc.robot.auto.actions.TurnInPlaceAction;
import frc.robot.util.geometry.Rotation2d;


public class Pigeon {

    private static Pigeon mInstance;

    public static Pigeon getInstance() {
        if (mInstance == null) {
            mInstance = new Pigeon(15);
        }
        return mInstance;
    }

    // Actual pigeon object
    private final Pigeon2 mGyro;

    // Configs
    private boolean inverted = Constants.SwerveConstants.invertGyro;
    private Rotation2d yawAdjustmentAngle = Rotation2d.identity();
    private Rotation2d rollAdjustmentAngle = Rotation2d.identity();

    private Pigeon(int port) {        
        mGyro = new Pigeon2(port);
        mGyro.configFactoryDefault();
    }

    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.inverse());
        if (inverted) {
            return angle.inverse();
        }
        return angle;
    }

    public Rotation2d getRoll() {
        return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.inverse());
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setYaw(double angleDeg) {
        yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
    }

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw());
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getPitch());
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getRoll());
    }

    public boolean isTipping() {
        Rotation2d rotation = getUnadjustedPitch();
        if(Math.abs(rotation.getDegrees()) > 2.5){
            return true;
        }
        return false;
    }
}
