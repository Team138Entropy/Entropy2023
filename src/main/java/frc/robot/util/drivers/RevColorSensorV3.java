package frc.robot.util.drivers;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
public class RevColorSensorV3 {
    ColorSensorV3 colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);

    public void updateSmartdashboard(){
        Color DetectedColor = colorSensorV3.getColor();
        SmartDashboard.putNumber("red color", DetectedColor.red);
        SmartDashboard.putNumber("green color", DetectedColor.green);
        SmartDashboard.putNumber("blue color", DetectedColor.blue);
        SmartDashboard.putString("Color", DetectedColor.toString());
    }
}
