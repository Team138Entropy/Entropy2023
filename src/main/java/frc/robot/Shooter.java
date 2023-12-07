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

        resetRetract();

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
    }

    private boolean retractionOver(){
        return retractTimer.hasElapsed(0.25);
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

    public void selectSpeed(double meters){
        
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
        TopEject.setControl(new DutyCycleOut(0));
        BottomEject.setControl(new DutyCycleOut(0));
        HighIntake.setControl(new DutyCycleOut(0));
        MidIntake.setControl(new DutyCycleOut(0));
        LowIntake.setControl(new DutyCycleOut(0));
    }

    // Stops everything except the intake, which continues normally
    private void Load(){
        TopEject.setControl(new DutyCycleOut(0));
        BottomEject.setControl(new DutyCycleOut(0));
        HighIntake.setControl(new DutyCycleOut(0));
        MidIntake.setControl(new DutyCycleOut(MidIntake.getRotorVelocity.getValue()));
        LowIntake.setControl(new DutyCycleOut(LowIntake.getRotorVelocity.getValue()));
    }

    private void revUp(){
        TopEject.setControl(new DutyCycleOut(TopEject.getRotorVelocity.getValue()));
        BottomEject.setControl(new DutyCycleOut(BottomEject.getRotorVelocity.getValue()));
        HighIntake.setControl(new DutyCycleOut(0));
        MidIntake.setControl(new DutyCycleOut(0));
        LowIntake.setControl(new DutyCycleOut(0));
    }

    private void retract(){
        TopEject.setControl(new DutyCycleOut(-0.1));
        BottomEject.setControl(new DutyCycleOut(-0.1));
        HighIntake.setControl(new DutyCycleOut(-0.1));
        MidIntake.setControl(new DutyCycleOut(-0.1));
        LowIntake.setControl(new DutyCycleOut(-0.1));
    }

    // Timer in switch statement
    private void resetRetract(){
        TopEject.setControl(new DutyCycleOut(0.1));
        BottomEject.setControl(new DutyCycleOut(0.1));
        HighIntake.setControl(new DutyCycleOut(0.1));
        MidIntake.setControl(new DutyCycleOut(0.1));
        LowIntake.setControl(new DutyCycleOut(0.1));
    }
}

/*
3 new functions:
- run everything reverse at a slow speed (get ball back down from shooter) ---------------------retract
- return balls back to their position after the previous function (ensure balls don't drop) ----resetBallPosition
- only run shooter (everything else zero, prepares to fire) ------------------------------------revUp
- don't run shooter, run anything else (only run intake) ---------------------------------------Load
 */