package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.RuntimeJsonMappingException;
import com.ctre.phoenix6.controls.DutyCycleOut;
import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;

public class Shooter { 

    public double targetTopEjectVelocity;
    public double targetBottomEjectVelocity; 
    public double targetHighIntakeVelocity; 
    public double targetMidIntakeVelocity; 
    public double targetLowIntakeVelocity; 
    public SpeedLookupConfig TargetedSpeeds;
    VelocityLookupTable velocityLookup = new VelocityLookupTable();

    private final Timer retractTimer;
    private final Timer resetTimer;

    //Setting the motors
    com.ctre.phoenix6.hardware.TalonFX TopEject = new TalonFX(6);
    TalonFX BottomEject1 = new TalonFX(5);
    TalonFX BottomEject2 = new TalonFX(4);
    TalonFX HighIntake = new TalonFX(3);
    TalonFX MidIntake = new TalonFX(2);
    TalonFX LowIntake = new TalonFX(1);

    public enum ShooterState {
        PreppedToFire,
        Loaded,
        Retracted,
        ResetRetraction,
        Running,
        Stopped
      };
      private ShooterState mShooterState;

    // Public function, allows somebody to set the grasper state. Should be the only public function relating to the shooter
    public void setShooterState(ShooterState shooterMode){      
        mShooterState = shooterMode;
    }

    public void update(){
        switch(mShooterState) {


        case PreppedToFire:

        revUp();

        break;


        case Loaded:

        Load();

        break;


        case Retracted:
        
        retractTimer.start();
        retract();
        if (retractionOver() == true){
            setShooterState(ShooterState.Stopped);
        }
        
        break;


        case ResetRetraction:
        
        resetTimer.start();
        resetRetract();
        if(resetOver() == true){
            setShooterState(ShooterState.Stopped);
        }

        break;
        

        case Stopped:

        Stop();

        break;


        case Running:

        Run();

        break;
        default:
        break;
        }

        //
    }

    private boolean retractionOver(){
        return retractTimer.hasElapsed(0.2);
    }

    private boolean resetOver(){
        return resetTimer.hasElapsed(0.2);
    }

    // Sets the distance (in meters) that the shooter is aiming for
    public void setTargetDistance(double meters){
        TargetedSpeeds = velocityLookup.getSpeedFromDistance(meters);
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

    // Sets the RPS of each individual motor
    private void Run(){
        TopEject.setControl(new VelocityVoltage(TargetedSpeeds.TopEject));
        BottomEject.setControl(new VelocityVoltage(TargetedSpeeds.BottomEject));
        HighIntake.setControl(new VelocityVoltage(TargetedSpeeds.HighIntake));
        MediumIntake.setControl(new VelocityVoltage(TargetedSpeeds.MidIntake));
        LowIntake.setControl(new VelocityVoltage(TargetedSpeeds.LowIntake));
    }

    // Stops all motors
    private void Stop(){
        updateDutyCycle(0, 0, 0, 0, 0);
    }

    // Stops everything except the intake, which continues normally
    private void Load(){
        updateDutyCycle(0, 0, 0, MidIntake.getRotorVelocity.getValue(), LowIntake.getRotorVelocity.getValue());
    }

    private void revUp(){
        updateDutyCycle(TopEject.getRotorVelocity.getValue(), BottomEject.getRotorVelocity.getValue(), 0, 0, 0);
    }

    private void retract(){
        updateDutyCycle(-0.1, -0.1, -0.1, -0.1, -0.1);
    }

    // Timer in switch statement
    private void resetRetract(){
        updateDutyCycle(0.1, 0.1, 0.1, 0.1, 0.1);
    }

    private void updateDutyCycle(double topShooter, double bottomShooter, double highLoad, double midLoad, double lowLoad){
        TopEject.setControl(new DutyCycleOut(topShooter));
        BottomEject.setControl(new DutyCycleOut(bottomShooter));
        HighIntake.setControl(new DutyCycleOut(highLoad));
        MidIntake.setControl(new DutyCycleOut(midLoad));
        LowIntake.setControl(new DutyCycleOut(lowLoad));
    }
}

/*
Set each button on a controller to a different state
Test in Sim
 */