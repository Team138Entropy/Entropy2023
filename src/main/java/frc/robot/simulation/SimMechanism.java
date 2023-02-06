package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

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

    // Sim Arm Length Constraints
    private final double mArmMinLength = .3;
    private final double mArmMaxLength = 2;
    private final double mArmMaxDelta = .05;
    private double mCurrentArmLength = mArmMinLength;

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
        mTower = mArmRoot.append(new MechanismLigament2d("tower", 3, 270));
        mTower.setColor(new Color8Bit(Color.kSilver));

        // Arm which pivots on top of the tower
        mArm = mArmRoot.append(new MechanismLigament2d("arm", mCurrentArmLength, 45));

        // Grasper Mechanism Canvas
        mGrasperMech = new Mechanism2d(9, 9); //Canvas Size (10,10)

        // Grasper Mechanism Root (Right Side)
        mGrasperRootRight = mGrasperMech.getRoot("grasper root right", 7, 2); 
        // Grasper Mechanism Root (Left Side)
        mGrasperRootLeft = mGrasperMech.getRoot("grasper root left", 2, 2); 

        // Grasper Connection Ligament (Left Side)
        mGrasperConnection = mGrasperRootLeft.append(new MechanismLigament2d("Grasper Connection", 5, 0));
        // Left Grasper Ligament
        mLeftClaw = mGrasperRootLeft.append(new MechanismLigament2d("Left Claw", 3, 90));
        // Right Grasper Ligament
        mRightClaw = mGrasperRootRight.append(new MechanismLigament2d("Right Claw", 3, 90));      

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

/*
  private final double mArmMinLength = .7;
    private final double mArmMaxLength = 2;
    private final double mArmMaxDelta = .05;
    currentArmLength
 */
    // Simulate Arm Extension
    public void SetArmLength (double length){
        // Convert actual arm encoder units to extension units
        // Calculate Arm Ranges
        double actualArmRange = (Constants.Arm.MaxExtensionPosition - Constants.Arm.MinExtensionPosition);
        double simArmRange = (mArmMaxLength - mArmMinLength);

        // Percentage this length takes up of arm
        double targetPercent = length/actualArmRange;

        // sim value to use based on that precentage
        double simLength = (targetPercent * simArmRange) + mArmMinLength;

        // Verify sim value is within range
        if(simLength >= mArmMinLength && simLength <= mArmMaxLength)
        {
            // Determine the sign of the difference
            double difference = mCurrentArmLength - simLength;
            double delta = Math.abs(difference);

            // Prevent arm from changing by more than the delta
            if(delta > mArmMaxDelta) {
                delta = mArmMaxDelta;

                // Convert Sign if this is subtraction
                if(difference > 0){
                    delta *= -1;
                }
                
                // Add (or subtract) delta
                mCurrentArmLength += delta;
            }else {
                // Arm Change is within delta, can just set it
                mCurrentArmLength = simLength;
            }

            // Set Arm Length
            mArm.setLength(mCurrentArmLength);
        }
    }

    // Close Grasper
    // Moves Grasper to close state
    public void closeGrasper(){
        mLeftClaw.setAngle(45);
        mRightClaw.setAngle(135);
    }

    // Open Grasper 
    // Move Graspers to open state
    public void openGrasper(){
        mLeftClaw.setAngle(90);
        mRightClaw.setAngle(90);
    }
    
    public void updateSmartDashboard() {
        final String key = "SimMechanism/";
        SmartDashboard.putData(key + "Arm Mechanism", mArmMech);
        SmartDashboard.putData(key + "Grasper Mechanism", mGrasperMech);
    }
}