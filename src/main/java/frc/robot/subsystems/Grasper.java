package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Grasper {
    private static Grasper mInstance;

    //Grasper pneumatics
    Solenoid GrasperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    //NEO Motor 
    CANSparkMax GrasperMotor = new CANSparkMax(15, MotorType.kBrushless);
    //Beam Sensor
    DigitalInput input = new DigitalInput(0);

    public static synchronized Grasper getInstance() {
        if (mInstance == null) {
          mInstance = new Grasper();
        }
        return mInstance;

    }

      

 public void setGrasperTrue(){
    GrasperSolenoid.set(true);
      }
   public void setGrasperFalse(){
       GrasperSolenoid.set(false);
      }

   public void setGrasperOpen(){
        setGrasperFalse();
        }
   public void setGrasperClosed(){
        setGrasperTrue();
     }

     public void setGrasperWheelIntake(){
     GrasperMotor.set(0.2);
     }
     public void cancelGrasperWheelIntake(){
      GrasperMotor.set(0);
      }

     public void updateSmartDashBoard(){
      //Grasper Motor
      SmartDashboard.putNumber("Voltage", GrasperMotor.getBusVoltage());
      SmartDashboard.putNumber("Temperature", GrasperMotor.getMotorTemperature());
      SmartDashboard.putNumber("Output", GrasperMotor.getAppliedOutput());
      //IR Beam Sensor 
      SmartDashboard.putBoolean("IR Beam Sensor value", input.get());
  }

  public boolean getBeamSensorBroken(){
    return !input.get();
  }

}
    

