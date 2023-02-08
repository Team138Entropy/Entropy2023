package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.drivers.EntropyCANSparkMax;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Grasper {
    private static Grasper mInstance;


    //Grasper pneumatics
    private final Solenoid GrasperSolenoid;
    //NEO Motor 
    private final EntropyCANSparkMax GrasperWheelMotor;
    //Beam Sensor
    private final DigitalInput BeamSensor;
    //Beam Timer
    private final Timer beamActivationTimer;
    private boolean BeamSensorOn = true;
    //Wheel Timer
    private final Timer wheelCancellationTimer;
    //Reverse Wheel Timer
    private final Timer wheelReverseTimer;
    //Dribble Timer
    private final Timer dribbleTimer;
    private final Timer dribbleTimer2;
    private boolean DribbleActive = false;
    //Grasper Open/Closed
    private boolean mGrasperOpen = false;
    //allows for a toggle of dribble
    private boolean mdribbleOn = true;
    

    // Grasper State
    public enum GrasperState {
      FullyClosed,
      Closed,
      Open,
      FullyOpen
    };
    public GrasperState mGrasperState;


    public static synchronized Grasper getInstance() {
        if (mInstance == null) {
          mInstance = new Grasper();
        }
        return mInstance;
    }

    private Grasper()
    {
      GrasperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Talons.Grasper.PCMId);
      GrasperWheelMotor = new EntropyCANSparkMax(Constants.Talons.Grasper.IntakeMotor, MotorType.kBrushless);
      BeamSensor = new DigitalInput(0);
      beamActivationTimer = new Timer();
      wheelCancellationTimer = new Timer();
      wheelReverseTimer = new Timer();
      dribbleTimer = new Timer();
      dribbleTimer2 = new Timer();
      mGrasperState = GrasperState.FullyClosed;
    }
  
   

  // Constant Update Function 
  public void update(){
    // Perform Logic based on the Grasper State
    switch(mGrasperState) {

      case FullyClosed:

      //dont run unless 1 sec is true, then run until .25 is true, then reset and start timer/refer to beginning

        GrasperSolenoid.set(true);
        cancelGrasperWheelIntake();
        mGrasperOpen = false;
        

        if (mdribbleOn == true) {
          if (DribbleActive == true){
          setGrasperWheelIntake();
          if (getGrasperDribbleTimer2() == true){
            DribbleActive = false;
            dribbleTimer.reset();
            dribbleTimer.start();
          }
        }

          if (DribbleActive == false){
            cancelGrasperWheelIntake();
            if (getGrasperDribbleTimer() == true){
            DribbleActive = true;
            dribbleTimer2.reset();
            dribbleTimer2.start();
        }
      }
    }

      break;

      case Closed:

        setGrasperWheelIntake();

        if (mGrasperOpen == true) {
          GrasperSolenoid.set(true);
          mGrasperOpen = false;
        }
        
        if (getGrasperTimeElapsed1() == true){
          mGrasperState = GrasperState.FullyClosed;
        }

      break;

      case Open:

      GrasperSolenoid.set(false);
      setGrasperWheelReverse();
      mGrasperOpen = true;

      if (getGrasperEjectTimeElapsed() == true){
        mGrasperState = GrasperState.FullyOpen;
      }

      break;
      case FullyOpen:
    
      GrasperSolenoid.set(false);
      setGrasperWheelIntake();
      mGrasperOpen = true;

      if (getBeamSensorBroken() == true){
        mGrasperState = GrasperState.Closed;
      }

      break;
      default:
      break;
    }
  }

  // Open the Grasper
   // Restart Beam Activation Timer
   public void setGrasperOpen(){
    mGrasperState = GrasperState.Open;
    beamActivationTimer.reset();
    beamActivationTimer.start();
    BeamSensorOn = false;
   }


   // Close the Grasper
   // Stop the Beam Activiation Timer
   // Allow the Grasper Motor to run for 1 Second to help pull in!
   public void setGrasperClosed(){
    mGrasperState = GrasperState.Closed;
    beamActivationTimer.stop();
   }

  // Start the Intake Motor
  public void setGrasperWheelIntake(){
    GrasperWheelMotor.set(0.2);
  }

  //Run the intake motor reversed
  public void setGrasperWheelReverse(){
    GrasperWheelMotor.set(-0.2);
  }

  // Stop the Intake Motor
  public void cancelGrasperWheelIntake(){
    GrasperWheelMotor.set(0);
  }


  // Has the Grasper stay open long enough to use beam sensor
  public boolean getGrasperTimeElapsed3(){
    return beamActivationTimer.hasElapsed(3);
  }
  // Timer for the wheels when closing the Grasper
  public boolean getGrasperTimeElapsed1(){
    return wheelCancellationTimer.hasElapsed(1);
  }
  //Timer for the wheels running in reverse
  public boolean getGrasperEjectTimeElapsed(){
    return wheelReverseTimer.hasElapsed(1);
  }
  //Timer for dribble
  public boolean getGrasperDribbleTimer(){
    return dribbleTimer.hasElapsed(1);
  }
  public boolean getGrasperDribbleTimer2(){
    return dribbleTimer2.hasElapsed(0.25);
  }

  //Checks if the beam sensors connection becomes broken
  public boolean getBeamSensorBroken(){
    if (BeamSensorOn == false){
      if (getGrasperTimeElapsed3() == true){
        BeamSensorOn = true;
      }
    }

    if (BeamSensorOn == true) {
      return !BeamSensor.get();
    }
    return false;
}

// Boolean made for Sim Code
public boolean grasperIsClosed(){
  if (mGrasperState == GrasperState.FullyClosed){
    return true;
  }
  else {
    return false;
  }
}

public boolean grasperIsOpen(){
  if (mGrasperState == GrasperState.Open){
    return true;
  }
  else {
    return false;
  }
}

//commands for sim code
public void setSimGrasperClosed(){
  mGrasperState = GrasperState.FullyClosed;
}

public void setSimGrasperOpen(){
  mGrasperState = GrasperState.Open;
}


//gets the current grasper state
public GrasperState getGrasperState(){
  return mGrasperState;
}


  public void updateSmartDashBoard(){
    final String key = "Grasper/";
    //Grasper Motor
    SmartDashboard.putNumber(key + "Voltage", GrasperWheelMotor.getBusVoltage());
    SmartDashboard.putNumber(key + "Temperature", GrasperWheelMotor.getMotorTemperature());
    SmartDashboard.putNumber(key + "Output", GrasperWheelMotor.getAppliedOutput());
    //IR Beam Sensor 
    SmartDashboard.putBoolean(key + "IR Beam Sensor", BeamSensor.get());
    //Solenoid
    SmartDashboard.putBoolean(key + "Solenoid State", GrasperSolenoid.get());
    SmartDashboard.putNumber(key + "Solenoid Channel", GrasperSolenoid.getChannel());
    // General 
    SmartDashboard.putString(key + "GrasperState", mGrasperState.toString());

    // Update Grasper Motor Smartdashboard
    GrasperWheelMotor.updateSmartdashboard();
  }
} 

