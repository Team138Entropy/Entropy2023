package frc.robot.util.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class EntropyTalonFX extends TalonFX {
    private final double decisecondsPerSecond = 10;
    private String deviceInfo;
    private String description;
    private double ticksPerMeter;
    private double saturationVoltage = 12;
    private int velocitySlotIdx;
    private int positionSlotIdx;
    private int lastSlot = 0;
    private boolean invertEncoder = false;


    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public EntropyTalonFX(int deviceNumber, double ticksPerMeter, int velocitySlotIdx, int positionSlotIdx) {
        super(deviceNumber);
        this.ticksPerMeter = ticksPerMeter;
        this.velocitySlotIdx = velocitySlotIdx;
        this.positionSlotIdx = positionSlotIdx;
        setDeviceInfo(deviceNumber);
    }

    public EntropyTalonFX(int deviceNumber){
        super(deviceNumber);
        this.ticksPerMeter = Constants.Drive.Encoders.ticksPerMeters;
        this.velocitySlotIdx = MotorConfigUtils.VELOCITY_SLOT_IDX;
        this.positionSlotIdx = MotorConfigUtils.POSITION_SLOT_IDX;
        setDeviceInfo(deviceNumber);
    }

    // Talon Name
    private void setDeviceInfo(int deviceNumber){
        deviceInfo = "Talon FX " + deviceNumber;
    }

    public void invertEncoder(){
        invertEncoder = !invertEncoder;
    }

    private double getInversionValue(){
        return invertEncoder ? -1.0 : 1.0;
    }

    public double getDistanceMeters() {
        return getInversionValue() * ((double) this.getSelectedSensorPosition() / ticksPerMeter);
    }

    public double getRateMetersPerSecond() {
        return getInversionValue() * ((double) this.getSelectedSensorVelocity() * decisecondsPerSecond / ticksPerMeter);
    }

    public double getEncoderTicks(){
        return (double) this.getSelectedSensorPosition();
    }

    public void zeroEncoder(){
        this.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }


    public void setPercent(double percent) {
        this.set(ControlMode.PercentOutput, percent);
    }

    public void setVelocityMetersPerSecond(double velocityMetersPerSecond) {
        if (lastSlot != velocitySlotIdx) selectProfileSlot(velocitySlotIdx, 0);
        this.set(ControlMode.Velocity, velocityMetersPerSecond * ticksPerMeter / decisecondsPerSecond);
    }

    public void setPositionMeters(double positionMeters) {
        if (lastSlot != positionSlotIdx) selectProfileSlot(positionSlotIdx, 0);
        this.set(ControlMode.Position, positionMeters * ticksPerMeter);
    }

    public void setVoltage(double voltage) {
        this.set(ControlMode.PercentOutput, voltage / saturationVoltage);
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        lastSlot = slotIdx;
        super.selectProfileSlot(slotIdx, pidIdx);
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        saturationVoltage = voltage;
        return super.configVoltageCompSaturation(voltage, timeoutMs);
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage) {
        saturationVoltage = voltage;
        return super.configVoltageCompSaturation(voltage);
    }

    public void setDescription(String desc)
    {
        description = desc;
    }

    public String getDescription(String desc)
    {
        return description;
    }

    public void updateSmartdashboard()
    {    
       SmartDashboard.putString(deviceInfo + "/Description", description);
       SmartDashboard.putNumber(deviceInfo + "/Voltage", getBusVoltage());
       SmartDashboard.putNumber(deviceInfo + "/Current", getSupplyCurrent()); 
       SmartDashboard.putBoolean(deviceInfo + "/Inverted", getInverted()); 
       SmartDashboard.putNumber(deviceInfo + "/ClosedLoopTarget", getClosedLoopTarget());
       SmartDashboard.putNumber(deviceInfo + "/ClosedLoopTarget", getClosedLoopError());
       SmartDashboard.putNumber(deviceInfo + "/MotorOutputPercent", getMotorOutputPercent());
       SmartDashboard.putNumber(deviceInfo + "/MotorOutputVoltage", getMotorOutputVoltage());
       SmartDashboard.putNumber(deviceInfo + "/FiremwareVersion", getFirmwareVersion());
       SmartDashboard.putNumber(deviceInfo + "/SelectedSensorPosition", getSelectedSensorPosition());
    }
}