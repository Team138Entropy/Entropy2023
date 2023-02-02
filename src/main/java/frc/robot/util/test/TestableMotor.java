package frc.robot.util.test;

import frc.robot.util.drivers.EntropyCANSparkMax;
import frc.robot.util.drivers.EntropyTalonFX;
import frc.robot.util.drivers.EntropyTalonSRX;

public class TestableMotor {

    //enum for all motor types, add more if needed
    public enum MotorType{
        TALON_SRX,
        TALON_FX,
        SPARK_MAX,
        NONE;
    }

    public MotorType testedMotor = MotorType.NONE;

    //encoder values
    public double minExpectedEncoderValue = 0;
    public double maxExpectedEncoderValue = 0;
    public double motorEncoderValue = 0;

    //current values
    public double minExpectedCurrent = 0;
    public double maxExpectedCurrent = 0;
    public double motorCurrent = 0;

    //3 contructors for each motor type
    public TestableMotor(EntropyTalonSRX talonSRX) {
        testedMotor = MotorType.TALON_SRX;
    }

    public TestableMotor(EntropyTalonFX talonFX) {
        testedMotor = MotorType.TALON_FX;
    }

    public TestableMotor(EntropyCANSparkMax sparkMax) {
        testedMotor = MotorType.SPARK_MAX;
    }

    //set expected and real encoder value functions
    public void setExpectedEncoderValue(double minExpectedValue, double maxExpectedValue){
        minExpectedEncoderValue = minExpectedValue;
        maxExpectedEncoderValue = maxExpectedValue;
    }

    public void setEncoderValue(double encoderValue){
        motorEncoderValue = encoderValue;
    }

    //set expected and real current value functions
    public void setExpectedCurrentValue(double maxExpectedCurrentValue, double minExpectedCurrentValue){
        minExpectedCurrent = minExpectedCurrentValue;
        maxExpectedCurrent = maxExpectedCurrentValue;
    }

    public void setCurrentValue(double current){
        motorCurrent = current;
    }

    //status functions check the real value compaired to the range of expected values and returns true if they are in the expected range
    public boolean getEncoderStatus(){
        if(motorEncoderValue >= minExpectedEncoderValue && motorEncoderValue <= maxExpectedEncoderValue){
            return true;
        }
        return false;
    }

    public boolean getCurrentStatus(){
        if(motorCurrent >= minExpectedCurrent && motorCurrent <= maxExpectedCurrent){
            return true;
        }
        return false;
    }

}
