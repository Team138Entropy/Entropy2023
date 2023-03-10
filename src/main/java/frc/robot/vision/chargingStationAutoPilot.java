package frc.robot.vision;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.util.drivers.Pigeon;
import frc.robot.util.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.TuneableNumber;

public class chargingStationAutoPilot {
    private static chargingStationAutoPilot mInstance;

    public static synchronized chargingStationAutoPilot getInstance() {
        if (mInstance == null) {
          mInstance = new chargingStationAutoPilot();
        }
        return mInstance;
    }
    private final Drive mDrive = Drive.getInstance();

    // pigeon sensor reference
    public Pigeon mPigeon = Pigeon.getInstance();
    public double pitchAngleDegrees = mPigeon.getUnadjustedPitch().getDegrees();
    public double pitchAngleRate = mPigeon.getUnadjustedPitchRate().getDegrees();

    public double error;

    private boolean isDone = false;

    public enum balanceState{
        LOOKING_FOR_PITCH(0),
        LEVELING(1),
        DRIVE_BACK(2),
        LEVELED(3);

        private final int _value;

        balanceState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public balanceState mState;

    //placeholder PID values
    private final PIDController balanceController = new PIDController(
        Constants.AutoPilot.CSAutoPilotKP.get(),
        Constants.AutoPilot.CSAutoPilotKI.get(),
        Constants.AutoPilot.CSAutoPilotKD.get()
    );


    private chargingStationAutoPilot() {

    }

    public void startBalance() {
        mState = balanceState.LOOKING_FOR_PITCH;
        isDone = false;
    }
    

    public void update() {
        

        switch(mState){
            case LOOKING_FOR_PITCH:
             isDone = false;
             if((Math.abs(pitchAngleDegrees) <= Math.abs(Constants.AutoPilot.ChargeStationDegreeThreshold.get())) || (Math.abs(pitchAngleRate) > Constants.AutoPilot.PitchAngleRateThreshold.get())){
                mState = balanceState.LEVELING;
             }else{
                mState = balanceState.LEVELED;
             }
                break;
            case LEVELING:
                error = balanceController.calculate(pitchAngleDegrees);
                error = MathUtil.clamp(error, Constants.AutoPilot.maxLevelSpeedLow.get(), Constants.AutoPilot.maxLevelSpeedHigh.get());
                // Set Swerve to those Module States
                mDrive.setSwerveDrive(new Translation2d(error,0), 0, true, true, false);
                if(Math.abs(pitchAngleRate) > Constants.AutoPilot.PitchAngleRateThreshold.get()){
                    mState = balanceState.DRIVE_BACK;
                }
                break;
            case DRIVE_BACK:
                //TODO: add drive back code!
                if((Math.abs(pitchAngleDegrees) <= Math.abs(Constants.AutoPilot.ChargeStationDegreeThreshold.get())) || (Math.abs(pitchAngleRate) > Constants.AutoPilot.PitchAngleRateThreshold.get())){
                    mState = balanceState.LEVELING;
                }else{
                    mState = balanceState.LEVELED;
                }
                break;
            case LEVELED:
                mDrive.setBrake(true);
                isDone = true;
                break;
        }

    }


    public boolean getDone() {
        return isDone;
    }


    public void updateSmartdashboard() {
        String key = "ChargingStation/";
        SmartDashboard.putNumber(key + "Pitch (Degrees)", mPigeon.getUnadjustedPitch().getDegrees());
        SmartDashboard.putNumber(key + "Pitch Rate (Degrees/Period)", mPigeon.getUnadjustedPitchRate().getDegrees());
        //SmartDashboard.putString(key + "current state", mState.toString());
        SmartDashboard.putNumber(key + "PID error", error);

    }

}
