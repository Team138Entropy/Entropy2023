package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm {
    private static Arm mInstance;
    TalonSRX ArmMotor = new TalonSRX(5);

    public static synchronized Arm getInstance() {
        if (mInstance == null) {
          mInstance = new Arm();
        }
        return mInstance;
      }
      private Arm () {
        ArmMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        ArmMotor.setSensorPhase(true);
        ArmMotor.setInverted(true);
        ArmMotor.config_kF(0, 1);
        ArmMotor.config_kP(0, 30);
        ArmMotor.config_kI(0, .01);
        ArmMotor.config_kD(0, 300);
        ArmMotor.configSelectedFeedbackCoefficient(360.0/8192.0);
        ArmMotor.configMotionAcceleration(5);
        ArmMotor.configMotionCruiseVelocity(5, 10);
        ArmMotor.setSelectedSensorPosition(90.0);

    }

    public double getGravity(){
        double currentRadians = ArmMotor.getSelectedSensorPosition() * Constants.Misc.degreeToRadian;
        double feedForward = 0.2 * Math.cos(currentRadians);
            return feedForward;
        

    }

    public void setArmAngle(double Degrees){
        double feedForward = getGravity();
        ArmMotor.set(ControlMode.MotionMagic, Degrees, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void updateSmartDashBoard(){
        SmartDashboard.putNumber("ArmPosition", ArmMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Percent Output", ArmMotor.getMotorOutputPercent());
    }



    


 public void setArmJog (double Percent){
     ArmMotor.set(ControlMode.PercentOutput, Percent);
 }
}

