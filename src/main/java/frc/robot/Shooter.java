package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import java.lang.Math;

public class Shooter { 

    public double targetTopEjectVelocity;
    public double targetBottomEjectVelocity; 
    public double targetHighIntakeVelocity; 
    public double targetMidIntakeVelocity; 
    public double targetLowIntakeVelocity; 

    //Setting the motors
    com.ctre.phoenix6.hardware.TalonFX TopEject = new TalonFX(6);
    TalonFX BottomEject1 = new TalonFX(5);
    TalonFX BottomEject2 = new TalonFX(4);
    TalonFX HighIntake = new TalonFX(3);
    TalonFX MidIntake = new TalonFX(2);
    TalonFX LowIntake = new TalonFX(1);

    // Interpolating Table
    

    // Sets the distance (in meters) that the shooter is aiming for
    public void setTargetDistance(double meters){

    }

    // Used in isReady - Determines if the current rps of the motor is within the acceptable range
    public boolean checkBounds(double leeway, double target, double actual){
        if (Math.abs(actual - target) <= leeway){
            return true;
        } return false;
    }

    // Determines if the RPS of each motor is correct for the set distance
    // There is a window for error coded in - the "leeway" variable determines the margin of error (in rps)
    public boolean isReady(){
        double topEjectVelocity = TopEject.getRotorVelocity.getValue();
        double bottomEjectVelocity = BottomEject.getRotorVelocity.getValue();
        double highIntakeVelocity = HighIntake.getRotorVelocity.getValue();
        double midIntakeVelocity = MidIntake.getRotorVelocity.getValue();
        double lowIntakeVelocity = LowIntake.getRotorVelocity.getValue();
        if (checkBounds(5, targetTopEjectVelocity, topEjectVelocity) &&
            checkBounds(5, targetBottomEjectVelocity, bottomEjectVelocity) &&
            checkBounds(5, targetHighIntakeVelocity, highIntakeVelocity) &&
            checkBounds(5, targetMidIntakeVelocity, midIntakeVelocity) &&
            checkBounds(5, targetLowIntakeVelocity, lowIntakeVelocity)){
        return true;
        } return false;
    }

    public void selectSpeed(double meters){
        
    }

    // Sets the RPS of each individual motor
    public void Run(){
        TopEject.setControl(new VelocityVoltage(selectSpeed));
        BottomEject1.setControl(new VelocityVoltage(selectSpeed));
        BottomTopEject2.setControl(new VelocityVoltage(selectSpeed));
        HighIntake.setControl(new VelocityVoltage(selectSpeed));
        MediumIntake.setControl(new VelocityVoltage(selectSpeed));
        LowIntake.setControl(new VelocityVoltage(selectSpeed));
    }

    // Stops all motors
    public void Stop(){
        TopEject.setControl(new DutyCycleOut(0));
        BottomEject1.setControl(new DutyCycleOut(0));
        BottomEject2.setControl(new DutyCycleOut(0));
        HighIntake.setControl(new DutyCycleOut(0));
        MidIntake.setControl(new DutyCycleOut(0));
        LowIntake.setControl(new DutyCycleOut(0));
    }

}