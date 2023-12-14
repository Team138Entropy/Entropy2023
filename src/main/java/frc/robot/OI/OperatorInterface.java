package frc.robot.OI;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.OI.APacController.*;
import frc.robot.OI.XboxController.Axis;
import frc.robot.OI.XboxController.Button;
import frc.robot.OI.XboxController.Side;


public class OperatorInterface {
    private static OperatorInterface mInstance;

    // Instances of the Driver and Operator Controller
    private XboxController mDriverController;



    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        mDriverController = new XboxController(0);
       
    }

    
    public double getDriveTurn() {
        return mDriverController.getJoystick(Side.RIGHT, Axis.X);
    }


    // SwerveTranslation that only allows movement in 1 direction
    public Translation2d getSimpleSwerveTranslation()
    {
        // X and Y Components
        double x = 0;
        double y = 0;
        if (mDriverController.getDPad() == 0) {
            x = 1; // Move Forward (UpField)
        }else if (mDriverController.getDPad() == 90) {
            y = -1; // Move Left
        }else if (mDriverController.getDPad() == 180) {
            x = -1; // Move Backwards (DownField)
        }else if (mDriverController.getDPad() == 270) {
            y = 1; // Move Right
        }
        return new Translation2d(x, y);
    }

  
    
    public boolean getZeroGyro(){
        return mDriverController.getButton(Button.BACK);
    }


    public boolean getFastBalance(){
        return mDriverController.getButton(Button.RB);
    }



    public boolean getDriveAutoSteer(){
        return mDriverController.getTrigger(Side.RIGHT);
    }

    public boolean getTeleopBrake(){
        return mDriverController.getButton(Button.LB);
    }


    public boolean getAutoPilotLeftStrafe(){
        return mDriverController.getDPad() == 270;
    }

    public boolean getAutoPilotRightStrafe(){
        return mDriverController.getDPad() == 90;
    }

    public boolean getDrivePrecisionSteer(){
        return mDriverController.getTrigger(Side.LEFT);
    }

    public boolean getDriveSportSteer(){
        return mDriverController.getButton(Button.RB);
    }

    public void setOperatorRumble(boolean a){ 
        //mOperatorController.setRumble(a);
    }

    public void setDriverRumble(boolean rumbleOn, double rumbleValue){ 
        mDriverController.setRumble(rumbleOn, rumbleValue);
    }


    /**
     * Switches the Robot Mode
     * On the Start Button of the Operator Controller
     * @return
     */
    public boolean getSwitchModePress(){
        //return mOperatorStartButton.update(mOperatorController.getButton(Button.START));
        return false;
    }




    public boolean getBallerBalling() {
        return mDriverController.getTrigger(Side.RIGHT);
    }











    public boolean getManualArmExtendUp() {
        //return manualExtensionup.update(mOperatorController2.getDPad() == 0);
        return false;
    }
    public boolean getManualArmExtendDown() {
        //return manualExtensionDown.update(mOperatorController2.getDPad() == 180);
        return false;
    }

    public boolean getManualArmRotateUp() {
        //return manualRotationUp.update(mOperatorController2.getDPad() == 90);
        return false;
    }

    public boolean getManualArmRotateDown() {
        //return manualRotationDown.update(mOperatorController2.getDPad() == 270);
        return false;
    }

    public boolean getIntakeOpen() {
        //return mOperatorController.getTrigger(Side.LEFT);
        return false;
    }

    public boolean getIntakeClose() {
        //return mOperatorController.getTrigger(Side.RIGHT);
        return false;
    }




    public boolean getArmAngleMinusOne(){
        //return testDecrementRotation.update(mOperatorController.getButton(Button.A));
        return false;
    }

    public boolean getArmAnglePlusFive(){
        //return testIncrimentRotation.update(mOperatorController.getButton(Button.X))
        return false;
    }

    public boolean getArmAngleMinusFive(){ 
        //return testDecrementRotation.update(mOperatorController.getButton(Button.B));
        return false;
    }

    public boolean setArmExtendedPlusOne(){
        //return testIncrimentExtension.update(0 == mOperatorController.getDPad());
        return false;
    }

    public boolean setArmExtendedMinusOne(){
        //return testDecrementExtension.update((180 == mOperatorController.getDPad()));
        return false;
    }

    public boolean setArmExtendedPlusFive(){
        //return testIncrimentExtension.update(270 == mOperatorController.getDPad());
        return false;
    }

    public boolean setArmExtendedMinusFive(){
        //return testDecrementExtension.update(90 == mOperatorController.getDPad());
        return false;
    }


    // Shooter code
    public boolean getShooterRunning(){
        return mDriverController.getTrigger(Side.RIGHT);
    }

    public boolean stopShooter(){
        return mDriverController.getTrigger(Side.LEFT);
    }

    public boolean revUpShooter(){
        return mDriverController.getButton(Button.X);
    }

    public boolean getShooterPrepped(){
        return mDriverController.getButton(Button.B);
    }

    public boolean getBallsRetracted(){
        return mDriverController.getButton(Button.Y);
    }

    public boolean getBallsReset(){
        return mDriverController.getButton(Button.A);
    }
}




