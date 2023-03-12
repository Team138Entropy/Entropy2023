package frc.robot.vision;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.util.drivers.Pigeon;
import frc.robot.util.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
    private final AutoPilot mAutoPilot = AutoPilot.getInstance();

    // pigeon sensor reference
    public Pigeon mPigeon = Pigeon.getInstance();
    public double pitchAngleDegrees = mPigeon.getUnadjustedPitch().getDegrees();
    public double pitchAngleRate = mPigeon.getUnadjustedPitchRate().getDegrees();

    public double error;
    private Pose2d mFinalRobotPose = new Pose2d();

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

    public balanceState mState = balanceState.LOOKING_FOR_PITCH;

    //placeholder PID values
    private final PIDController balanceController = new PIDController(
        Constants.AutoPilot.CSAutoPilotKP.get(),
        Constants.AutoPilot.CSAutoPilotKI.get(),
        Constants.AutoPilot.CSAutoPilotKD.get()
    );


    private chargingStationAutoPilot() {

    }

    public void startBalance() {
        mState = balanceState.LEVELING;
        isDone = false;
    }
    

    public void update() {    
        // Calcuate Pitch Rate and Degrees 
        pitchAngleDegrees = mPigeon.getUnadjustedPitch().getDegrees();
        pitchAngleRate = mPigeon.getUnadjustedPitchRate().getDegrees();

        // Update State
        switch(mState){
            case LOOKING_FOR_PITCH:
            // Drive Backward Very Slowly Looking for Pitch
            // This will slowly bring the robot onto the charging station
            mDrive.setSwerveDrive(new Translation2d(-.45,0), 0, true, true, false);

             isDone = false;
             if((Math.abs(pitchAngleDegrees) >= Math.abs(Constants.AutoPilot.ChargeStationDegreeThreshold.get()))){
                // Charing Station has been Reached
                mState = balanceState.LEVELING;
             }
                break;
            case LEVELING:
                error = balanceController.calculate(pitchAngleDegrees);
                // If this drives the wrong way, multiple by -1 
                error *= -1;
                error = MathUtil.clamp(error, Constants.AutoPilot.maxLevelSpeedLow.get(), Constants.AutoPilot.maxLevelSpeedHigh.get());
                // Set Swerve to those Module States
                mDrive.setSwerveDrive(new Translation2d(error,0), 0, true, true, false);
                if(Math.abs(pitchAngleRate) > Constants.AutoPilot.PitchAngleRateThreshold.get()){
                    mState = balanceState.DRIVE_BACK;

                    // Setup AutoPilot 
                    mAutoPilot.setUseDriveOnlyPose(true);

                    // Calculate the current robot pose, and create a slight translation
                    // This last little movement will return tuning
                    Pose2d currRobotPose = mAutoPilot.getCurrentPose();
                    double offset = -.55;
                    mFinalRobotPose = new Pose2d(
                        currRobotPose.getTranslation().plus(new Translation2d(offset, 0)),
                        currRobotPose.getRotation()
                    );

                    // Tell Auto Pilot and Reset
                    mAutoPilot.setTargetPose(mFinalRobotPose);
                    mAutoPilot.reset();
                }
                break;
            case DRIVE_BACK:
                // Drive until AutoPilot is complete
                mAutoPilot.update();
                if(mAutoPilot.atGoal())
                {
                    mState = balanceState.LEVELED;
                }
                break;
            case LEVELED:
                // Level is Complete
                mDrive.setSwerveDrive(new Translation2d(0,0), 0, true, true, false);
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
        SmartDashboard.putString(key + "State", mState.toString());
        SmartDashboard.putNumber(key + "PID error", error);
        SmartDashboard.putString(key + "Final Robot Pose", mFinalRobotPose.toString());

    }

}
