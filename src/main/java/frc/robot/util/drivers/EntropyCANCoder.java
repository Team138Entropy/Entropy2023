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
        SmartDashboard.putNumber(deviceInfo + "/FirmwareVersion", getFirmwareVersion());
        SmartDashboard.putNumber(deviceInfo + "/AbsolutePosition", getAbsolutePosition());
        SmartDashboard.putNumber(deviceInfo + "/Position", getPosition());
        SmartDashboard.putNumber(deviceInfo + "/Velocity", getVelocity());
    }
}
