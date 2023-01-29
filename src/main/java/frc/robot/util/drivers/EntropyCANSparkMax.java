package frc.robot.util.drivers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EntropyCANSparkMax extends CANSparkMax {
    private String deviceInfo;
    private String description;

    public EntropyCANSparkMax(int deviceNumber, MotorType mtype)
    {
        super(deviceNumber, mtype);
        setDeviceInfo(deviceNumber);
    }

    private void setDeviceInfo(int deviceNumber){
        deviceInfo = "CANSparkMax " + deviceNumber;
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
      // SmartDashboard.putString(deviceInfo + "/Description", description);
       SmartDashboard.putNumber(deviceInfo + "/Voltage", getBusVoltage());
       SmartDashboard.putNumber(deviceInfo + "/Applied Output", getAppliedOutput());
       SmartDashboard.putBoolean(deviceInfo + "/Inverted", getInverted()); 
       SmartDashboard.putNumber(deviceInfo + "/FirmwareVersion", getFirmwareVersion());
       SmartDashboard.putNumber(deviceInfo + "/Motor Temperature", getMotorTemperature());
    }
}
