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

    private Mechanism2d mGrasperMech;
    private MechanismRoot2d mGrasperRootRight;
    private MechanismRoot2d mGrasperRootLeft;
    private MechanismLigament2d mGrasperConnection;
    private MechanismLigament2d mLeftClaw;
    private MechanismLigament2d mRightClaw;

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
        mArmRoot = mArmMech.getRoot("arm", 1.5, 1.5);
        // Tower of the Mechanism
        mTower = mArmRoot.append(new MechanismLigament2d("tower", .8, 270));

        // Arm which pivots on top of the tower
        mArm = mArmRoot.append(new MechanismLigament2d("arm", 1, 45));
        

        // Grasper Mechanism Canvas
        mGrasperMech = new Mechanism2d(9, 9); //Canvas Size (10,10)

        // Grasper Mechanism Root (Right Side)
        mGrasperRootRight = mGrasperMech.getRoot("grasper root right", 7, 2); 
        // Grasper Mechanism Root (Left Side)
        mGrasperRootLeft = mGrasperMech.getRoot("grasper root left", 2, 2); 

        // Grasper Connection Ligament (Left Side)
        mGrasperConnection = mGrasperRootLeft.append(new MechanismLigament2d("Grasper Connection", 5, 0));
        // Left Grasper Ligament
        mLeftClaw = mGrasperRootLeft.append(new MechanismLigament2d("Left Claw", 5, 90));
        // Right Grasper Ligament
        mRightClaw = mGrasperRootRight.append(new MechanismLigament2d("Right Claw", 5, 90));
        

        // Useful Methods on mArm:
        //mArm.setAngle(0);
        //mArm.setLength(0);

    }
public void SetArmAngle (double angle ){
    if(angle <= 0 && angle >= -360){
        angle = angle +360;

    }
        
    if(angle <= 230 || angle >= 310){
        mArm.setAngle(angle);
    }

}

public void SetArmLength (double length){
    if(length <= 1.5 && length >= 1 ){
            mArm.setLength(length);
        }
    }

    
public void simGrasperState (double angle){
    if(angle <= 0 && angle >= -360){
        angle = angle +360;

    }
        
    if(angle <= 230 || angle >= 310){
        mArm.setAngle(angle);
    }
}

/*
if (mOperatorInterface.getSimGrasperClosed()){
    mGrasper.setSimGrasperClosed();
}
*/

// make it so the mode changes stuff not buttons

    public void updateSmartDashboard() {
        final String key = "SimMechanism/";
        SmartDashboard.putData(key + "Arm Mechanism", mArmMech);
        SmartDashboard.putData(key + "Grasper Mechanism", mGrasperMech);
    }
}