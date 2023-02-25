package frc.robot.OI;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Enums.*;
import frc.robot.OI.APacController.*;
import frc.robot.OI.XboxController.Axis;
import frc.robot.OI.XboxController.Button;
import frc.robot.OI.XboxController.Side;
import frc.robot.util.LatchedBoolean;
import frc.robot.vision.photonVision;

public class OperatorInterface {
    private static OperatorInterface mInstance;

    // Instances of the Driver and Operator Controller
    private XboxController mDriverController;
    private APacController mOperatorController;
    private APacController mOperatorController2;
    // Latched Booleans
    private LatchedBoolean mOperatorSelectButton = new LatchedBoolean();
    private LatchedBoolean mOperatorStartButton = new LatchedBoolean();
    private LatchedBoolean mLeftBumper = new LatchedBoolean();
    private LatchedBoolean mArmRotateUp = new LatchedBoolean();
    private LatchedBoolean mArmRotateDown = new LatchedBoolean();
    private LatchedBoolean mClimberTestPress = new LatchedBoolean();
    private LatchedBoolean mOperatorClimbApprovePress = new LatchedBoolean();

    private LatchedBoolean mExtensionUp = new LatchedBoolean();
    private LatchedBoolean mExtensionDown = new LatchedBoolean();
    private LatchedBoolean mExtensionSwitchMode = new LatchedBoolean();

    private LatchedBoolean manualRotationUp = new LatchedBoolean();
    private LatchedBoolean manualRotationDown = new LatchedBoolean();
    private LatchedBoolean manualExtensionup = new LatchedBoolean();
    private LatchedBoolean manualExtensionDown = new LatchedBoolean();

    private LatchedBoolean balanceDrive = new LatchedBoolean();

    // Test Position Mode Control
    private LatchedBoolean testIncrimentRotation = new LatchedBoolean();
    private LatchedBoolean testDecrementRotation = new LatchedBoolean();
    private LatchedBoolean testIncrimentExtension = new LatchedBoolean();
    private LatchedBoolean testDecrementExtension = new LatchedBoolean();

    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        mDriverController = new XboxController(Constants.Controllers.Driver.port);
        mOperatorController = new APacController(Constants.Controllers.Operator.port);
        mOperatorController2 = new APacController(Constants.Controllers.Operator2.port);
    }

    public boolean getClimberTest(){
        return mClimberTestPress.update(mDriverController.getButton(Button.B));
    }

    public boolean getClimberTest2(){
        return mClimberTestPress.update(mDriverController.getButton(Button.Y));
    }

    public double getDriveThrottle() {
        return mDriverController.getJoystick(Side.LEFT, Axis.Y);
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

    /* Swerve Drive Controls */
    public Translation2d getSwerveTranslation() {
        // joystick inputs
        double forwardAxis = mDriverController.getJoystick(Side.LEFT, Axis.Y);
        double strafeAxis = mDriverController.getJoystick(Side.LEFT, Axis.X);


        SmartDashboard.putNumber("Swerve Forward Axis", forwardAxis);
        SmartDashboard.putNumber("Swerve Strafe Axis", strafeAxis);

        forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);
        //Translation2d tAxes = new Translation2d(strafeAxis, forwardAxis);

        // optional code to deadband each one side


        if (Math.abs(tAxes.getNorm()) < Constants.Controllers.joystickDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadband_vector = new Translation2d(Constants.Controllers.joystickDeadband, deadband_direction);

            double scaled_x = tAxes.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double scaled_y = tAxes.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            return new Translation2d(scaled_x, scaled_y).times(Constants.SwerveConstants.maxSpeed);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = mDriverController.getJoystick(Side.RIGHT, Axis.X);
        if(getDrivePrecisionSteer()){
            rotAxis *= .4;
        }else if(getDriveSportSteer()){
            rotAxis *= .5;
        }
        rotAxis = Constants.SwerveConstants.invertRotateAxis ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < Constants.Controllers.joystickDeadband) {
            return 0.0;
        } else {
            return Constants.SwerveConstants.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * Constants.Controllers.joystickDeadband)) / (1 - Constants.Controllers.joystickDeadband);
        }
    }

    // Get Quick Snap to a Direction
    public SwerveCardinal getSwerveSnap() {
        SwerveCardinal result = SwerveCardinal.NONE;

        // Forward, Backwards, Left, Right
        if (mDriverController.getButton(Button.Y)) {
            result = SwerveCardinal.FORWARDS;
        }
        if (mDriverController.getButton(Button.A)) {
            result = SwerveCardinal.BACKWARDS;
        }
        if (mDriverController.getButton(Button.X)) {
            result = SwerveCardinal.LEFT;
        }
        if (mDriverController.getButton(Button.B)) {
            result = SwerveCardinal.RIGHT;
        }
        return result;
    }

 

    // Returns the Targeted Position on the Field
    public TargetedPositions getScoringCommand() {
        TargetedPositions node = TargetedPositions.NONE;

        if (mOperatorController2.getButton(Buttons.SW1)) {
            node = TargetedPositions.GRID_1;
        }else if (mOperatorController2.getButton(Buttons.SW2)) {
            node = TargetedPositions.GRID_2;
        }else if (mOperatorController2.getButton(Buttons.SW3)) {
            node = TargetedPositions.GRID_3;
        }else if (mOperatorController2.getButton(Buttons.SW4)) {
            node = TargetedPositions.GRID_4;
        }else if (mOperatorController2.getButton(Buttons.SW5)) {
            node = TargetedPositions.GRID_5;
        }else if (mOperatorController2.getButton(Buttons.SW6)) {
            node = TargetedPositions.GRID_6;
        }else if (mOperatorController2.getButton(Buttons.SW7)) {
            node = TargetedPositions.GRID_7;
        }else if (mOperatorController2.getButton(Buttons.SW8)) {
            node = TargetedPositions.GRID_8;
        }else if (mOperatorController2.getButton(Buttons.COIN)) {
            node = TargetedPositions.GRID_9;
        }else if (mOperatorController2.getAxis(Axises.X) == 1 || mOperatorController2.getAxis(Axises.X) == -1) {
            node = TargetedPositions.SUBSTATION_RIGHT;
        }else if (mOperatorController2.getAxis(Axises.Y) == 1 || mOperatorController2.getAxis(Axises.Y) == -1) {
            node = TargetedPositions.SUBSTATION_LEFT;
        }

        return node;
    }

    public boolean getResetOdometry(){
        return mDriverController.getButton(Button.BACK);
    }
    
    public boolean getZeroGyro(){
        return mDriverController.getButton(Button.BACK);
    }


    public boolean getFastBalance(){
        return mDriverController.getButton(Button.RB);
    }

    public boolean getBalanceMode(){
        return balanceDrive.update(mDriverController.getButton(Button.START));
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
        return mDriverController.getTrigger(Side.RIGHT);
    }

    public void setOperatorRumble(boolean a){ 
        //mOperatorController.setRumble(a);
    }

    public void setDriverRumble(boolean rumbleOn, double rumbleValue){ 
        mDriverController.setRumble(rumbleOn, rumbleValue);
    }

    public boolean getArmRotateUp() {
          return mArmRotateUp.update(mDriverController.getButton(Button.Y));
    }
    
    public boolean getArmEject() {
        return mOperatorController2.getButton(Buttons.SW2);
    }

    public boolean getGrasperIntakeManual() {
        return mLeftBumper.update(mOperatorController.getButton(Buttons.SW1));
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

    public boolean getSelectButtonPress(){
        //return mOperatorSelectButton.update(mOperatorController.getButton(Button.BACK));
        return false;
    }

    public boolean getArmEncoderZero(){
        //return mOperatorController.getButton(Button.R_JOYSTICK);
        return false;
    }

    public boolean getArmExtend() {
        //int input = mOperatorController.getDPad();
        //return input <= 45 && input >= 315;
        return false;
    }

    public boolean getArmRetract() {
        //int input = mOperatorController.getDPad();
        //return input <= 225 && input >= 135;
        return false;
    }

    public boolean getArmExtendManual() {
        //return mOperatorController.getButton(Button.B);
        return false;
    }

    public boolean getArmRetractManual() {
        //return mOperatorController.getButton(Button.X);
        return false;
    }

    public boolean getArmExtendPress(){
       // return mExtensionUp.update(mOperatorController.getButton(Button.B));
       return false;
    }

    public boolean getArmRetractPress(){
        //return mExtensionDown.update(mOperatorController.getButton(Button.X));
        return false;
    }

    public boolean getArmJogForward() {
        return mDriverController.getButton(Button.Y);
    }

    public boolean getArmJogBackward() {
        return mDriverController.getButton(Button.A);
    }

    public boolean getArmJogMidForward() {
        return mDriverController.getButton(Button.X);
    }
    
    public boolean getArmJogMidBackward() {
        return mDriverController.getButton(Button.B);
    }

    public boolean getArmJogExtended5() {
        //return mOperatorController.getButton(Button.RB);
        return false;
    }



    public boolean getArmJogExtended() {
        //return mOperatorController.getButton(Button.RB);
        return mOperatorController.getAxis(Axises.X) > .25;
    }
    
    public boolean getArmJogRetracted() {
        // return mDriverController.getButton(Button.A);
         return mOperatorController.getAxis(Axises.X) < -.25;
     } 

    public boolean getArmExtended2() {
        return (0 == mDriverController.getDPad());
    }

    public boolean getArmExtended4() {
        return (90 == mDriverController.getDPad());
    }

    public boolean getArmExtended6() {
        return (180 == mDriverController.getDPad());
    }

    public boolean getArmExtended0() {
        return (270 == mDriverController.getDPad());
    }

    public boolean getArmRotateForward() {
        return mOperatorController2.getButton(Buttons.SW1);
    }

    public boolean getArmRotateBackward() {
        return mOperatorController2.getButton(Buttons.SW2);
    }

    public boolean getGrasperModeSwap() {
        return mDriverController.getTrigger(Side.RIGHT);
    }
    public boolean getGrasperWheelIntake() {
        //return mDriverController.getTrigger(Side.LEFT);
        return false;
    }

    public boolean getWristUp() {
        return mDriverController.getTrigger(Side.RIGHT);
    }
  
    public boolean getFeedDown() {
        return mDriverController.getButton(Button.A);
    }

    public ArmTargets getArmTarget() {
        ArmTargets target = ArmTargets.NONE;

        if(mOperatorController2.getAxis(Axises.X) == 1 || mOperatorController2.getAxis(Axises.Y) == -1){
            target = ArmTargets.INTAKE_FRONT;
        }else if(mOperatorController2.getAxis(Axises.X) == -1 || mOperatorController2.getAxis(Axises.Y) == 1){
            target = ArmTargets.INTAKE_BACK;
        }else if(mOperatorController.getButton(Buttons.SW4)){
            target = ArmTargets.HOME_BACKSIDE;
        }else if(mOperatorController.getButton(Buttons.SW3)){
            target = ArmTargets.HOME_FRONTSIDE;
        }else if(mOperatorController.getButton(Buttons.COIN)){
            target = ArmTargets.TOP_SCORING_FRONT;
        }else if(mOperatorController.getAxis(Axises.X) == 1){
            target = ArmTargets.MID_SCORING_FRONT;
        }else if(mOperatorController.getAxis(Axises.Y) == 1){
            target = ArmTargets.LOW_SCORING_FRONT;
        }else if(mOperatorController.getAxis(Axises.X) == -1){
            target = ArmTargets.MID_SCORING_BACK;
        }else if(mOperatorController.getAxis(Axises.Y) == -1){
            target = ArmTargets.LOW_SCORING_BACK;
        }else if(mOperatorController2.getButton(Buttons.A)){
            target = ArmTargets.INTAKE_GROUND_FRONT;
        }else if(mOperatorController2.getButton(Buttons.B)){
            target = ArmTargets.INTAKE_GROUND_BACK;
        }
        return target;
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

    public boolean getDriverRelease() {
        return mDriverController.getTrigger(Side.RIGHT);
    }

    public boolean getModeSwitch() {
        return mOperatorController2.getButton(Buttons.SW3);
    }

    public boolean getGrasperOpen() {
        return mOperatorController.getButton(Buttons.SW2);
    
    }

    public boolean getGrasperClosed() {
        return mOperatorController.getButton(Buttons.SW1);
    }




    public boolean getArmAnglePlusOne(){
        return testIncrimentRotation.update(mOperatorController2.getButton(Buttons.SW5));
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

    public TargetedObject setTargetedObject(){
        if(mOperatorController.getButton(Buttons.B)){
            return TargetedObject.CONE;
        }
        return TargetedObject.CUBE;
    }
}



