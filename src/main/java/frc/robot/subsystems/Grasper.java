package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.drivers.BeamSensor;
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
    private final BeamSensor mBeamSensor;
    //Beam Timer
    private final Timer beamActivationTimer;
    private boolean BeamSensorOn = true;
    //Wheel Timer
    private final Timer wheelCancellationTimer;
    //Grasper Open/Closed
    private boolean mGrasperOpen = false;

    // Grasper State
    public enum GrasperState {
      FullyClosed,
      Closed,
      Open
    };
    private GrasperState mGrasperState;


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
      mBeamSensor = new BeamSensor(0);
      beamActivationTimer = new Timer();
      wheelCancellationTimer = new Timer();
      mGrasperState = GrasperState.FullyClosed;
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

  // Stop the Intake Motor
  public void cancelGrasperWheelIntake(){
    GrasperWheelMotor.set(0);
  }

  // Constant Update Function 
  public void update(){
    // Perform Logic based on the Grasper State
    switch(mGrasperState) {

      case FullyClosed:

        GrasperSolenoid.set(true);
        cancelGrasperWheelIntake();
        mGrasperOpen = false;

      break;

      case Closed:

        setGrasperWheelIntake();

        if (mGrasperOpen == true) {
          mGrasperOpen = false;
        }
        GrasperSolenoid.set(true);
        
        if (getGrasperTimeElapsed1() == true){
          mGrasperState = GrasperState.FullyClosed;
        }

      break;
      case Open:
        // TODO - Do we need to do anything here?
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

  // Has the Grasper stay open long enough to use beam sensor
  public boolean getGrasperTimeElapsed3(){
    return beamActivationTimer.hasElapsed(3);
  }
  // Timer for the wheels when closing the Grasper
  public boolean getGrasperTimeElapsed1(){
    return wheelCancellationTimer.hasElapsed(1);
  }

  public boolean getBeamSensorBroken(){
    if (BeamSensorOn == false){
      if (getGrasperTimeElapsed3() == true){
        BeamSensorOn = true;
      }
    }

    if (BeamSensorOn == true) {
      return !mBeamSensor.get();
    }
    return false;
  }

  // Get Current Grasper State
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
    SmartDashboard.putBoolean(key + "IR Beam Sensor", mBeamSensor.get());
    //Solenoid
    SmartDashboard.putBoolean(key + "Solenoid State", GrasperSolenoid.get());
    SmartDashboard.putNumber(key + "Solenoid Channel", GrasperSolenoid.getChannel());
    // General 
    SmartDashboard.putString(key + "GrasperState", mGrasperState.toString());

    // Update Grasper Motor Smartdashboard
    GrasperWheelMotor.updateSmartdashboard();
  }
} 