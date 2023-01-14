package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grasper {
    private static Grasper mInstance;

    TalonSRX GrasperMotor = new TalonSRX(7);
    Solenoid GrasperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

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

       }
    

