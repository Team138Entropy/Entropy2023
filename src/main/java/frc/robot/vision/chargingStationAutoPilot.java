package frc.robot.vision;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.util.drivers.Pigeon;
import frc.robot.util.geometry.Rotation2d;
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

    public double rollAngleDegrees = mPigeon.getRoll().getDegrees();

    public double pitchAngleDegrees = mPigeon.getUnadjustedPitch().getDegrees();
    public double pitchAngleRate = mPigeon.getUnadjustedPitchRate().getDegrees();

    public boolean autoBalanceXMode = false;

    public double respondRate = 0.01;

    private boolean isDone = false;

    public int stage = 0;

    public Timer mTimer = new Timer();


    //placeholder PID values
    private final PIDController balanceController = new PIDController(
        Constants.AutoPilot.CSAutoPilotKP.get(),
        Constants.AutoPilot.CSAutoPilotKI.get(),
        Constants.AutoPilot.CSAutoPilotKD.get());

    static final double chargingStationDegreeThreshold = 4;

    double xAxisRate = 0;

    double yAxisRate = 0;

    private chargingStationAutoPilot() {

    }


    // Reset Done Status
    public void reset() {
        stage = 0;
        isDone = false;
    } 
    
    // Is Balance COmplete
    public boolean isDone() {
        return isDone;
    }

    public void update(boolean leftStrafe, boolean rightStrafe) {
        balanceController.setP(Constants.AutoPilot.CSAutoPilotKP.get());
        balanceController.setI(Constants.AutoPilot.CSAutoPilotKI.get());
        balanceController.setD(Constants.AutoPilot.CSAutoPilotKD.get());

        // Get Pitch Angle and Pitch Rate
        pitchAngleDegrees = mPigeon.getUnadjustedPitch().getDegrees();
        pitchAngleRate = mPigeon.getUnadjustedPitchRate().getDegrees();
        xAxisRate = 0;
        autoBalanceXMode = false;
        if((Math.abs(pitchAngleDegrees) <= Math.abs(chargingStationDegreeThreshold))
         || (Math.abs(pitchAngleRate) > 11)) 
        {
            // Pitch Angle Rate is Greater than 15 or Angle is less than 5 Degree THreashold
            // Stop! Were done!
            stage = 1;
            autoBalanceXMode = false;
            mTimer.reset();
            mTimer.start();
        } else if(stage == 0) {
            autoBalanceXMode = true;
            xAxisRate = balanceController.calculate(pitchAngleDegrees);
            xAxisRate = xAxisRate*respondRate;
        }else if(stage == 1){
            if(mTimer.hasElapsed(1.6)){
                stage = 2;
                autoBalanceXMode = false;
            }else{
                xAxisRate = .01;
                autoBalanceXMode = true;
            }
        }


        SmartDashboard.putBoolean("autoBalanceX mode", autoBalanceXMode);
        SmartDashboard.putNumber("chargeStation threshold", chargingStationDegreeThreshold);
        SmartDashboard.putNumber("pitch angle", pitchAngleDegrees);
        


        if ( autoBalanceXMode ) {
            /*
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
            */
            //idk if this is how you use a pid controller
            xAxisRate = balanceController.calculate(pitchAngleDegrees);
            xAxisRate = xAxisRate*respondRate;

        }

        if (leftStrafe) {
            yAxisRate = .3;
        }else if (rightStrafe){
            yAxisRate = -.3;
        }else{
            yAxisRate = 0;
        }

        if(autoBalanceXMode){
            mDrive.setBrake(false);

            // Still balancing
            // Set Speeds into Swerve System
            ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(.4, yAxisRate, 0);

            // Call Autonomous Chasis Speed 
            var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

            // Set Swerve to those Module States
            mDrive.setModuleStates(targetSwerveModuleStates);
        } else {
            // Stop balancing
            mDrive.setBrake(true);
            mDrive.setSwerveDrive(new Translation2d(), 0, true, true, false);

        }


        SmartDashboard.putNumber("xAxisRate", xAxisRate);
    }

    private void stop() {

        // Set Speeds into Swerve System
        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(0, 0, 0);

        // Call Autonomous Chasis Speed 
        var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

        // Set Swerve to those Module States
        mDrive.setModuleStates(targetSwerveModuleStates);
    }

    public boolean getDone(){
        return stage == 2;
    }


    public void updateSmartdashboard()
    {
        String key = "ChargingStation/";
        SmartDashboard.putNumber(key + "Pitch (Degrees)", mPigeon.getUnadjustedPitch().getDegrees());
        SmartDashboard.putNumber(key + "Pitch Rate (Degrees/Period)", mPigeon.getUnadjustedPitchRate().getDegrees());

    }

}
