package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grasper {
    private static Grasper mInstance;

    TalonSRX GrasperMotor = new TalonSRX(7);
    Solenoid GrasperCubeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    Solenoid GrasperConeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    public static synchronized Grasper getInstance() {
        if (mInstance == null) {
          mInstance = new Grasper();
        }
        return mInstance;
      }

      public void setCubeGrasperTrue(){
    GrasperCubeSolenoid.set(true);
      }
      public void setCubeGrasperFalse(){
        GrasperConeSolenoid.set(false);
          }
    
        public void setConeGrasperTrue(){
    GrasperConeSolenoid.set(true);
         }
        public void setConeGrasperFalse(){
        GrasperConeSolenoid.set(false);
                 }

      public void setGrasperOpen(){
        setConeGrasperFalse();
        setCubeGrasperFalse();
        }

        public void setCubeGrasperClosed(){
            setCubeGrasperTrue();
         }
         public void setConeGrasperClosed(){
            setConeGrasperTrue();
         }

       }
    

