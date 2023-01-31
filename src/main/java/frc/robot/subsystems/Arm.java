package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Enums.ArmTargets;
import frc.robot.util.drivers.EntropyTalonSRX;

public class Arm extends Subsystem {
    private static Arm mInstance;

    private final EntropyTalonSRX MasterShoulderMotor;
    private final EntropyTalonSRX SecondaryShoulderMotor;
    private final EntropyTalonSRX ExtensionMotor;

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
        // Primary Motor -> Positive Percent Output should increase position -> 0:360
        //                  Negative Percent Output should decrease position -> 360:0
        // Secondary Motor -> Is physically inverted and should simply follow the Primary Motor
        MasterShoulderMotor.configFactoryDefault();
        SecondaryShoulderMotor.configFactoryDefault();
        MasterShoulderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyClosed);
        MasterShoulderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyClosed);
        MasterShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        MasterShoulderMotor.setSensorPhase(false);
        MasterShoulderMotor.setInverted(true);
        MasterShoulderMotor.config_kF(0, 1);
        MasterShoulderMotor.config_kP(0, 30);
        MasterShoulderMotor.config_kI(0, .01);
        MasterShoulderMotor.config_kD(0, 300);
        MasterShoulderMotor.configSelectedFeedbackCoefficient(360.0/8192.0);
        MasterShoulderMotor.configMotionAcceleration(20);
        MasterShoulderMotor.configMotionCruiseVelocity(20, 10);
        SecondaryShoulderMotor.follow(MasterShoulderMotor); // Secondary Motor will follow Primary Motor
        SecondaryShoulderMotor.setInverted(true);


        // Extension Motor Configuration
        ExtensionMotor.configFactoryDefault();
        ExtensionMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyClosed);
        ExtensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyClosed);
        ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        ExtensionMotor.setInverted(true);
        ExtensionMotor.setSensorPhase(true);
        ExtensionMotor.config_kF(0, 1);
        ExtensionMotor.config_kP(0, 0.9);
        ExtensionMotor.config_kI(0, 0);
        ExtensionMotor.config_kD(0, 0);
        ExtensionMotor.configMotionAcceleration(10000);
        ExtensionMotor.configMotionCruiseVelocity(5000);
    }

    // Gets the Feed Forward Value based on Gravity
    //      90 & 270 are straight up and straight down. No KF
    //      0 & 180 are completely horizontal. Maximum KF
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

    // Se the Extension Postion in Position Mode
    public void setArmExtension(double Inches){
        // TODO: Figure out Scaling Value
        // TODO: arm extension should factor angle?
        //ExtensionMotor.set(ControlMode.MotionMagic, Inches, DemandType.ArbitraryFeedForward, 0.3); 
    }

    // Get the current arm angle
    public double getArmAngle()
    {
        return MasterShoulderMotor.getSelectedSensorPosition();
    }

    // Get the current arm extension
    public double getArmExtension()
    {
        return ExtensionMotor.getSelectedSensorPosition();
    }

    // Test method to jog extension
    public void setExtensionJog (double Percent){
        ExtensionMotor.set(ControlMode.PercentOutput, Percent);
    }

    // Test method to shoulder jog
    public void setShoulderJog (double Percent){
        MasterShoulderMotor.set(ControlMode.PercentOutput, Percent);
    }

    public void updateSmartDashBoard(){

        //Arm Positioning and Extension
        final String key = "Arm/";
        SmartDashboard.putNumber(key + "Shoulder Position", getArmAngle());
        SmartDashboard.putNumber(key + "Shoulder Percent Output", MasterShoulderMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(key + "Shoulder Secondary Percent Output", SecondaryShoulderMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(key + "Extension Position", getArmExtension());
        SmartDashboard.putNumber(key + "Extension Percent Output", ExtensionMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(key + "Extension Per Output", ExtensionMotor.getMotorOutputPercent());
        MasterShoulderMotor.updateSmartdashboard();
        SecondaryShoulderMotor.updateSmartdashboard();
        ExtensionMotor.updateSmartdashboard();
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        MasterShoulderMotor.setSelectedSensorPosition(ArmTargets.START.armAngle);
        ExtensionMotor.setSelectedSensorPosition(ArmTargets.START.armExtend);
    }

    @Override
    public void checkSubsystem() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void testSubsystem() {
        // TODO Auto-generated method stub
        
    }
 }

 
