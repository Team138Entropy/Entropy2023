package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.drivers.EntropyTalonSRX;

public class Arm {
    private static Arm mInstance;

    EntropyTalonSRX MasterShoulderMotor;
    EntropyTalonSRX SecondaryShoulderMotor;
    EntropyTalonSRX ExtensionMotor;

    public static synchronized Arm getInstance() {
        if (mInstance == null) {
          mInstance = new Arm();
        }
        return mInstance;
      }

      private Arm () {
        // Init Talons
        MasterShoulderMotor = new EntropyTalonSRX(Constants.Talons.Arm.ShoulderMasterId);
        SecondaryShoulderMotor = new EntropyTalonSRX(Constants.Talons.Arm.ShoulderSlaveId);
        ExtensionMotor = new EntropyTalonSRX(Constants.Talons.Arm.ExtensionId);

        // Shoulder Motor Configuration
        MasterShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        MasterShoulderMotor.setSensorPhase(true);
        MasterShoulderMotor.setInverted(true);
        MasterShoulderMotor.config_kF(0, 1);
        MasterShoulderMotor.config_kP(0, 30);
        MasterShoulderMotor.config_kI(0, .01);
        MasterShoulderMotor.config_kD(0, 300);
        MasterShoulderMotor.configSelectedFeedbackCoefficient(360.0/8192.0);
        MasterShoulderMotor.configMotionAcceleration(5);
        MasterShoulderMotor.configMotionCruiseVelocity(5, 10);
        MasterShoulderMotor.setSelectedSensorPosition(90.0);
        SecondaryShoulderMotor.follow(MasterShoulderMotor); // Secondary Motor will follow Primary Motor
        SecondaryShoulderMotor.setInverted(false); // Secondary Motor is Inverted

        // Extension Motor Configuration
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
        double currentRadians = MasterShoulderMotor.getSelectedSensorPosition() * Constants.Misc.degreeToRadian;
        double feedForward = 0.2 * Math.cos(currentRadians);
            return feedForward;
    }

    // Set the Arm Angle in Position Mode
    public void setArmAngle(double Degrees){
        double feedForward = getGravity();
        MasterShoulderMotor.set(ControlMode.MotionMagic, Degrees, DemandType.ArbitraryFeedForward, feedForward);
        }
    }

    public void updateSmartDashBoard(){
        //Arm Positioning and Extension
        final String key = "Arm/";
        SmartDashboard.putNumber(key + "Shoulder Position", MasterShoulderMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber(key + "Shoulder Percent Output", MasterShoulderMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(key + "Extension Position", ExtensionMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber(key + "Extension Percent Output", ExtensionMotor.getMotorOutputPercent());
        MasterShoulderMotor.updateSmartdashboard();
        SecondaryShoulderMotor.updateSmartdashboard();
        ExtensionMotor.updateSmartdashboard();
    }

    public void setArmExtension(double Inches){
        // TODO: Figure out Scaling Value
        ExtensionMotor.set(ControlMode.MotionMagic, Inches, DemandType.ArbitraryFeedForward, 0.3); 
    }

    public void setExtensionJog (double Percent){
        ExtensionMotor.set(ControlMode.PercentOutput, Percent);
    }

    public void setShoulderJog (double Percent){
        MasterShoulderMotor.set(ControlMode.PercentOutput, Percent);
    }
 }

 
