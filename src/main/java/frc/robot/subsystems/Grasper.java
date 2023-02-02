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
import frc.robot.util.test.TestableMotor;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Grasper extends Subsystem{
    private static Grasper mInstance;


    //Grasper pneumatics
    private final Solenoid GrasperSolenoid;
    //NEO Motor 
    private final EntropyCANSparkMax GrasperWheelMotor;
    //Beam Sensor
    private final DigitalInput BeamSensor;
    //timer
    private final Timer beamActivationTimer;

    private boolean BeamSensorOn = true;

    public final TestableMotor TestGrasperMotor;

    // Grasper State
    private enum GrasperState {
      Closed,
      Closing,
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
      BeamSensor = new DigitalInput(0);
      beamActivationTimer = new Timer();
      TestGrasperMotor = new TestableMotor(GrasperWheelMotor);
      mGrasperState = GrasperState.Closed;
    }
  
   // Open the Grasper
   // Restart Beam Activation Timer
   public void setGrasperOpen(){
    GrasperSolenoid.set(false);
    beamActivationTimer.reset();
    beamActivationTimer.start();
    setGrasperWheelIntake();
    BeamSensorOn = false;
   }

   // Close the Grasper
   // Stop the Beam Activiation Timer
   // Allow the Grasper Motor to run for 1 Second to help pull in!
   public void setGrasperClosed(){
    GrasperSolenoid.set(true);
    cancelGrasperWheelIntake();
    beamActivationTimer.stop();
   }

  // Start the Intake Motor
  public void setGrasperWheelIntake(){
    GrasperWheelMotor.set(0.5);
  }

  // Stop the Intake Motor
  public void cancelGrasperWheelIntake(){
    GrasperWheelMotor.set(0);
  }

  // Constant Update Function 
  public void update(){
    // Perform Logic based on the Grasper State
    switch(mGrasperState) {
      case Closed:
        // TODO - Grasper is Closed. Do we need to do anything?

      break;
      case Closing:
        // TODO - Grasper is closing (air is pushing cylinders out)..what do we do

      break;
      case Open:
        // TODO - Do we need to do anything here?

      break;
      default:
      break;
    }
  }

  // Has the Grasper been open long enough to use beam sensor
  public boolean getGrasperTimeElapsed(){
    return beamActivationTimer.hasElapsed(3);
  }

  public boolean getBeamSensorBroken(){
    // if disabled, return false ... beam is never broken
    if (BeamSensorOn == false){
      if (getGrasperTimeElapsed() == true){
        BeamSensorOn = true;
      }
    }

    if (BeamSensorOn == true) {
      return !BeamSensor.get();
    }
    return false;
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

  @Override
  public void zeroSensors() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void checkSubsystem() {
    // TODO Auto-generated method stub
    
  }

} 

