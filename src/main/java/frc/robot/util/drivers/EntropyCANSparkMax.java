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
        setDescription("");
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
       final String key = "CANSparkMax/" + deviceInfo;
       /*
       SmartDashboard.putString(key + "/Description", description);
       SmartDashboard.putNumber(key + "/Voltage", getBusVoltage());
       SmartDashboard.putNumber(key + "/Applied Output", getAppliedOutput());
       SmartDashboard.putBoolean(key + "/Inverted", getInverted()); 
       SmartDashboard.putNumber(key + "/FirmwareVersion", getFirmwareVersion());
       SmartDashboard.putNumber(key + "/Motor Temperature", getMotorTemperature());
       */
    }
}
