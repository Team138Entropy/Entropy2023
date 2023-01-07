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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;



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

    // Simulation Motors

    // PID
    private double mAngleKp;
    private double mAngleKi;
    private double mAngleKd;
    private SimpleMotorFeedforward mFeedforward;

    public SwerveModule(int moduleNumber, SwerveModuleConstants swerveConstants) {
        mModuleNumber = moduleNumber;
        mModuleName = "Module " + moduleNumber;
        mModuleString = swerveConstants.moduleName;
        mAngleOffset = swerveConstants.angleOffset;
        mDriveMotor = new EntropyTalonFX(swerveConstants.driveMotorID);
        mAngleMotor = new EntropyTalonFX(swerveConstants.angleMotorID);
        mDesiredState = new SwerveModuleState(0, new Rotation2d());

        // Angle Encoder
        mAngleEncoder = new EntropyCANCoder(swerveConstants.cancoderID);
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

    public void zeroEncoders()
    {
        
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
        SmartDashboard.putNumber(BaseKey + "CanCoder Position", getCanCoder().getDegrees());

        // CANCoder
        mAngleEncoder.updateSmartdashboard();

        // Falcon500s
        mDriveMotor.updateSmartdashboard();
        mAngleMotor.updateSmartdashboard();
    }
}