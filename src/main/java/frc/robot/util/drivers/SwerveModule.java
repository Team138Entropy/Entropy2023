package frc.robot.util.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.drivers.CTRE.CTREConfigs;
import frc.robot.util.drivers.CTRE.CTREModuleState;
import frc.robot.util.math.Conversions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/*
 Swerve Module
 Each Swerve Module consists of a Drive Motor, Angle Motor and a CANCoder

 CANCoders in the swerve module measure absolute angle by reading the orientation of a cylindrical magnet
 glued to the swerve module in line with the rotational axis. Since the rotation of the magnet relative to
 the wheel will vary between module to module. An offset is needed to tell the robot the difference between the 
 magnet roation and wheel rotation. THere is no correlation with modules, its about how its mounted.

 To calculate, turn all bevels (to the right) of the front side.. so the wheels are facing front. Use a straight
 edge to line them up. Its important that the wheels are pointed in the same direction.

 TalonFX position is realtive to where the encoder is zeroed. Zero the TalonFX encoders on bootup 
 to the CANCoder reading minus the offset. So Requesting 0-360 degrees from the motor will set the appropriate angle.

 @TODO: pid constants of angle motor seem a little rough, might need a retune 
*/
public class SwerveModule  {
    private final int mModuleNumber;
    private final String mModuleName;
    private final String mModuleString;
    private final EntropyCANCoder mAngleEncoder;
    private double mLastAngle;
    private double mAngleOffset;

    // States
    private SwerveModuleState mDesiredState;

    // Motors
    private final EntropyTalonFX mDriveMotor;
    private final EntropyTalonFX mAngleMotor;

    // PID
    private double mAngleKp;
    private double mAngleKi;
    private double mAngleKd;
    private SimpleMotorFeedforward mFeedforward;

    // Simulation Only
    private SwerveModulePosition mSimPosition;


    public SwerveModule(int moduleNumber, SwerveModuleConstants swerveConstants) {
        mModuleNumber = moduleNumber;
        mModuleName = "Module " + moduleNumber;
        mModuleString = swerveConstants.moduleName;
        mAngleOffset = swerveConstants.angleOffset;
        mDriveMotor = new EntropyTalonFX(swerveConstants.driveMotorID, swerveConstants.CanBusID);
        mAngleMotor = new EntropyTalonFX(swerveConstants.angleMotorID, swerveConstants.CanBusID);
        mDesiredState = new SwerveModuleState(0, new Rotation2d()); // zero desired state

        // Feedforward Controller for non open loop
        mFeedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, 
                                            Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);


        // Set Motor Descriptions for logging
        mDriveMotor.setDescription(mModuleName + " Drive Motor");
        mAngleMotor.setDescription(mModuleName + " Angle Motor");

        // Angle Encoder
        mAngleEncoder = new EntropyCANCoder(swerveConstants.cancoderID, swerveConstants.CanBusID);
        configAngleEncoder();
        mAngleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        mAngleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

        // Angle Motor
        configAngleMotor();
        TalonFXConfiguration angleConfiguration = CTREConfigs.swerveAngleFXConfig();
        mAngleKp = angleConfiguration.slot0.kP;
        mAngleKi = angleConfiguration.slot0.kI;
        mAngleKd = angleConfiguration.slot0.kD;

        // Drive Motor
        configDriveMotor();

        // Simulation
        mSimPosition = new SwerveModulePosition();

        mLastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
        mDesiredState = CTREModuleState.optimize(desiredState, getState().angle); 

        if(isOpenLoop){
            double percentOutput = mDesiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(mDesiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, mFeedforward.calculate(mDesiredState.speedMetersPerSecond));
        }

        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angle = (Math.abs(mDesiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? mLastAngle : mDesiredState.angle.getDegrees(); 
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio)); 
        mLastAngle = angle;
    }

    public SwerveModuleState getDesiredState()
    {
        return mDesiredState;
    }

    public void resetToAbsolute(){
        
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - mAngleOffset, Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        mAngleEncoder.configFactoryDefault();
        mAngleEncoder.configAllSettings(CTREConfigs.swerveCancoderConfig());
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(CTREConfigs.swerveAngleFXConfig());
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig());
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public void updateAnglePID(double kP, double kI, double kD) {
        if (mAngleKp != kP) {
            mAngleKp = kP;
            mAngleMotor.config_kP(0, mAngleKp, Constants.CAN.kLongCANTimeoutMs);
        }
        if (mAngleKi != kI) {
            mAngleKi = kI;
            mAngleMotor.config_kI(0, mAngleKi, Constants.CAN.kLongCANTimeoutMs);
        }
        if (mAngleKd != kP) {
            mAngleKd = kD;
            mAngleMotor.config_kD(0, mAngleKd, Constants.CAN.kLongCANTimeoutMs);        
        }
    }

    public double[] getAnglePIDValues() {
        double[] values = {mAngleKp, mAngleKi, mAngleKd};
        return values;
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(mAngleEncoder.getAbsolutePosition());
    }

    public double getTargetAngle() {
        return mLastAngle;
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public int getModuleNumber()
    {
        return mModuleNumber;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
    }
    
    public double getHeadingDegrees() {
        return mAngleMotor.getSelectedSensorPosition() * Constants.SwerveConstants.Motor.kTurnDistancePerPulse;
      }
    
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    public double getDriveMetersPerSecond() {
        return mDriveMotor.getSelectedSensorVelocity() * Constants.SwerveConstants.Motor.kDriveDistancePerPulse * 10;
    }
    
    public double getDriveMeters() {
        return mDriveMotor.getSelectedSensorPosition() * Constants.SwerveConstants.Motor.kDriveDistancePerPulse;
    }

    public void zeroEncoders()
    {
        
    }

    public void updateSimPosition(double dt)
    {
        updateSimPosition(dt);
    }

    public void updateSimPosition(double dt, boolean invert)
    {
        mSimPosition.distanceMeters += ((mDesiredState.speedMetersPerSecond * (invert ? -1 : 1)) * dt);
        mSimPosition.angle = mDesiredState.angle;
    }

    public SwerveModulePosition getSimPosition()
    {
        return mSimPosition;
    }

    public void updateSmartDashBoard()
    { 
        String BaseKey = "Swerve Modules/" + mModuleName + "/";
        SmartDashboard.putString(BaseKey + "Description", mModuleString);
        SmartDashboard.putNumber(BaseKey + "DriveMotorID", mDriveMotor.getDeviceID());
        SmartDashboard.putNumber(BaseKey + "AngleMotorID", mAngleMotor.getDeviceID());
        SmartDashboard.putNumber(BaseKey + "CANCoderID", mAngleEncoder.getDeviceID());
        SmartDashboard.putNumber(BaseKey + "Last Angle", mLastAngle);
        SmartDashboard.putString(BaseKey + "Desired State", mDesiredState.toString());
        SmartDashboard.putString(BaseKey + "Sim Position", mSimPosition.toString());
        SmartDashboard.putNumber(BaseKey + "Drive Motor M/S", getDriveMetersPerSecond());
        SmartDashboard.putNumber(BaseKey + "Drive Motor Meters", getDriveMeters());
        SmartDashboard.putNumber(BaseKey + "CanCoder Position", getCanCoder().getDegrees());

        // CANCoder
        mAngleEncoder.updateSmartdashboard();

        // Falcon500s
        mDriveMotor.updateSmartdashboard();
        mAngleMotor.updateSmartdashboard();
    }
}