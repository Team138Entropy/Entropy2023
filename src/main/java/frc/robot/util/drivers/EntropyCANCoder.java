package frc.robot.util.drivers;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EntropyCANCoder extends CANCoder {
    private String deviceInfo;

    public EntropyCANCoder(int deviceNumber) {
        this(deviceNumber, "");
    }

    public EntropyCANCoder(int deviceNumber, String canBusId)
    {
        super(deviceNumber, canBusId);
        setDeviceInfo(deviceNumber);
    }

    private void setDeviceInfo(int deviceNumber){
        deviceInfo = "CANCoder " + deviceNumber;
    }

    public void updateSmartdashboard()
    {  
        final String key = "CanCoders/" + deviceInfo;
        SmartDashboard.putNumber(key + "/FirmwareVersion", getFirmwareVersion());
        SmartDashboard.putNumber(key + "/AbsolutePosition", getAbsolutePosition());
        SmartDashboard.putNumber(key + "/AbsolutePosition360", 360 - getAbsolutePosition());
        SmartDashboard.putNumber(key + "/Position", getPosition());
        SmartDashboard.putNumber(key + "/Velocity", getVelocity());
    }
}
