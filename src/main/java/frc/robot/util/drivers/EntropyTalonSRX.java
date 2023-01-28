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
