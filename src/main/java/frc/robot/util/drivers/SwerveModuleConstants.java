package frc.robot.util.drivers;

public class SwerveModuleConstants {
    public final String moduleName;
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(
        String modName, int driveMotorID, 
        int angleMotorID, int canCoderID, double angleOffset) {
        this.moduleName = modName;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}