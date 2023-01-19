package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimMechanism {
    private static SimMechanism mInstance;

    public static synchronized SimMechanism getInstance() {
        if (mInstance == null) {
          mInstance = new SimMechanism();
        }
        return mInstance;
    }

    private Mechanism2d mArmMech;
    private MechanismRoot2d mArmRoot;
    private MechanismLigament2d mTower;
    private MechanismLigament2d mArm;

    private SimMechanism()
    {
        // Init Mechanism 
        init();
    }

    private void init()
    {
        // Arm Mechanism Canvas
        mArmMech = new Mechanism2d(3, 3); //Canvas Size (3,3)

        // Arm Mechanism Root
        mArmRoot = mArmMech.getRoot("arm", 1.5, 0);
        // Tower of the Mechanism
        mTower = mArmRoot.append(new MechanismLigament2d("tower", .8, 90));

        // Arm which pivots on top of the tower
        mArm = mArmRoot.append(new MechanismLigament2d("arm", 1, 45));


        // Useful Methods on mArm:
        //mArm.setAngle(0);
        //mArm.setLength(0);

        // unsure how to control where it pivots
    }


    public void updateSmartDashboard() {
        final String key = "SimMechanism/";
        SmartDashboard.putData(key + "Arm Mechanism", mArmMech);
    }
}
