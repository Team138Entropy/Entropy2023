package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class Shooter { 
    

    //Setting the motors
    com.ctre.phoenix6.hardware.TalonFX TopEject = new TalonFX(6);
    TalonFX BottomEject1 = new TalonFX(5);
    TalonFX BottomEject2 = new TalonFX(4);
    TalonFX HighIntake = new TalonFX(3);
    TalonFX MidIntake = new TalonFX(2);
    TalonFX LowIntake = new TalonFX(1);

    // Sets the distance (in meters) that the shooter is aiming for
    public void setTargetDistance(double meters){

    }

    // Determines if the RPS of each motor is correct for the set distance
    // There is a window for error coded in - the motors do not have to be perfectly at their set RPS
    public boolean isReady(){
        TopEject.getRotorVelocity.getValue();
        BottomEject.getRotorVelocity.getValue();
        //4 is a placeholder 
        if (TopEject.getRotorVelocity >= 3.8 && TopEject.getRotorVelocity <= 4.2){
        return true;
        }
    }

    // Sets the RPS of each individual motor
    public void Run(){
        TopEject.setControl(new VelocityVoltage(_______));
        BottomEject1.setControl(new VelocityVoltage(_______));
        BottomTopEject2.setControl(new VelocityVoltage(_______));
        HighIntake.setControl(new VelocityVoltage(_______));
        MediumIntake.setControl(new VelocityVoltage(_______));
        LowIntake.setControl(new VelocityVoltage(_______));
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
