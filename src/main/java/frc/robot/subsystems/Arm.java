package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm {
    private static Arm mInstance;
    TalonSRX ShoulderMotor = new TalonSRX(5);

    TalonSRX ExtensionMotor = new TalonSRX(6);

    public static synchronized Arm getInstance() {
        if (mInstance == null) {
          mInstance = new Arm();
        }
        return mInstance;
      }
      private Arm () {
     ShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
     ShoulderMotor.setSensorPhase(true);
     ShoulderMotor.setInverted(true);
     ShoulderMotor.config_kF(0, 1);
     ShoulderMotor.config_kP(0, 30);
     ShoulderMotor.config_kI(0, .01);
     ShoulderMotor.config_kD(0, 300);
     ShoulderMotor.configSelectedFeedbackCoefficient(360.0/8192.0);
     ShoulderMotor.configMotionAcceleration(5);
     ShoulderMotor.configMotionCruiseVelocity(5, 10);
     ShoulderMotor.setSelectedSensorPosition(90.0);
     ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
     ExtensionMotor.setInverted(true);
     ExtensionMotor.setSensorPhase(true);
     ExtensionMotor.config_kF(0, 1);
     ExtensionMotor.config_kP(0, 0.9);
     ExtensionMotor.config_kI(0, 0);
     ExtensionMotor.config_kD(0, 0);
     ExtensionMotor.configMotionAcceleration(10000);
     ExtensionMotor.configMotionCruiseVelocity(5000);
     ExtensionMotor.setSelectedSensorPosition(0);
    }

    public double getGravity(){
        double currentRadians = ShoulderMotor.getSelectedSensorPosition() * Constants.Misc.degreeToRadian;
        double feedForward = 0.2 * Math.cos(currentRadians);
            return feedForward;
        

    }

    public void setArmAngle(double Degrees){
        double feedForward = getGravity();
     ShoulderMotor.set(ControlMode.MotionMagic, Degrees, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void updateSmartDashBoard(){
        SmartDashboard.putNumber("ArmPosition", ShoulderMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Percent Output", ShoulderMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("ExtensionPosition", ExtensionMotor.getSelectedSensorPosition());
    }

    public void setArmExtension(double Inches){
     ExtensionMotor.set(ControlMode.MotionMagic, Inches * 8600, DemandType.ArbitraryFeedForward, 0.3); 
     //extension takes inches
    }

    public void setExtensionJog (double Percent){
        ExtensionMotor.set(ControlMode.PercentOutput, Percent);
    }

 public void setShoulderJog (double Percent){
     ShoulderMotor.set(ControlMode.PercentOutput, Percent);
 }
}

