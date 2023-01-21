package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Grasper {
    private static Grasper mInstance;

    private boolean BeamSensorOn = true;

    //Grasper pneumatics
    Solenoid GrasperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    //NEO Motor 
    CANSparkMax GrasperWheelMotor = new CANSparkMax(15, MotorType.kBrushless);
    //Beam Sensor
    DigitalInput BeamSensor = new DigitalInput(0);
    //timer
    Timer beamActivationTimer = new Timer();


    public static synchronized Grasper getInstance() {
        if (mInstance == null) {
          mInstance = new Grasper();
        }
        return mInstance;

    }

   public void setGrasperOpen(){
    GrasperSolenoid.set(false);
    beamActivationTimer.reset();
    beamActivationTimer.start();
    setGrasperWheelIntake();
    BeamSensorOn = false;
     }

   public void setGrasperClosed(){
    GrasperSolenoid.set(true);
    cancelGrasperWheelIntake();
    beamActivationTimer.stop();
     }

    //setBeamEnabled(boolean enabled)
    // 

     public void setGrasperWheelIntake(){
     GrasperWheelMotor.set(0.2);
     }
     public void cancelGrasperWheelIntake(){
      GrasperWheelMotor.set(0);
      }

      public boolean getGrasperTimeElapsed3(){
        return beamActivationTimer.hasElapsed(3);
      }

     public void updateSmartDashBoard(){
      //Grasper Motor
      SmartDashboard.putNumber("Voltage", GrasperWheelMotor.getBusVoltage());
      SmartDashboard.putNumber("Temperature", GrasperWheelMotor.getMotorTemperature());
      SmartDashboard.putNumber("Output", GrasperWheelMotor.getAppliedOutput());
      //IR Beam Sensor 
      SmartDashboard.putBoolean("IR Beam Sensor value", BeamSensor.get());
  }

  public boolean getBeamSensorBroken(){
    // if disabled, return false ... beam is never broken
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
} 

