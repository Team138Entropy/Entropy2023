package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Wrist {
    private static Wrist mInstance;

    Solenoid WristSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    public static synchronized Wrist getInstance() {
        if (mInstance == null) {
          mInstance = new Wrist();
        }
        return mInstance;
      }

    public void setWristUp(){
    WristSolenoid.set(false);
      }
      public void setWristDown(){
     WristSolenoid.set(true);
      }
    
}
