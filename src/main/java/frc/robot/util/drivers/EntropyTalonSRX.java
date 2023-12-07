package frc.robot.util.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EntropyTalonSRX extends TalonSRX{
    private String deviceInfo;
    private String description;

    public EntropyTalonSRX(int deviceNumber) {
        super(deviceNumber);
        setDeviceInfo(deviceNumber);
        setDescription("");
    }

    // Talon Name
    private void setDeviceInfo(int deviceNumber){
        deviceInfo = "Talon SRX " + deviceNumber;
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
       final String key = "TalonSRXs/" + deviceInfo;
       SmartDashboard.putString(key + "/Description", description);
       SmartDashboard.putNumber(key + "/Voltage", getBusVoltage());
       SmartDashboard.putNumber(key + "/Current", getSupplyCurrent()); 
       SmartDashboard.putBoolean(key + "/Inverted", getInverted()); 
       SmartDashboard.putNumber(key + "/ClosedLoopTarget", getClosedLoopTarget());
       SmartDashboard.putNumber(key + "/ClosedLoopError", getClosedLoopError());
       SmartDashboard.putNumber(key + "/MotorOutputPercent", getMotorOutputPercent());
       SmartDashboard.putNumber(key + "/MotorOutputVoltage", getMotorOutputVoltage());
       SmartDashboard.putNumber(key + "/FiremwareVersion", getFirmwareVersion());
       SmartDashboard.putNumber(key + "/SelectedSensorPosition", getSelectedSensorPosition());
       SmartDashboard.putNumber(key + "/SelectedSensorVelocity", getSelectedSensorVelocity());
    }

}
