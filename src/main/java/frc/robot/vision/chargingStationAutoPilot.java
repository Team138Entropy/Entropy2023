package frc.robot.vision;

import frc.robot.subsystems.Drive;
import frc.robot.util.drivers.Pigeon;
import frc.robot.util.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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

    public boolean autoBalanceXMode = false;

    //placeholder PID values
    private final PIDController balanceController = new PIDController(0, 0, 0);


    static final double chargingStationDegreeThreshold = 5;

    double xAxisRate = 0;

    double yAxisRate = 0;

    private chargingStationAutoPilot() {

    }

    private void start() {
        
    }

    public void update(boolean leftStrafe, boolean rightStrafe) {
        

        if (!autoBalanceXMode && (Math.abs(pitchAngleDegrees) >= Math.abs(chargingStationDegreeThreshold))) {
            autoBalanceXMode = true;
        }
        else if (autoBalanceXMode && (Math.abs(pitchAngleDegrees) <= Math.abs(chargingStationDegreeThreshold))) {
            autoBalanceXMode = false;
        }


        if ( autoBalanceXMode ) {
            /*
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
            */
            //idk if this is how you use a pid controller
            xAxisRate = balanceController.calculate(pitchAngleDegrees);
        }

        if (leftStrafe) {
            yAxisRate = .3;
        }else if (rightStrafe){
            yAxisRate = -.3;
        }else{
            yAxisRate = 0;
        }

        


        // Set Speeds into Swerve System
        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(xAxisRate, yAxisRate, 0);

        // Call Autonomous Chasis Speed 
        var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

        // Set Swerve to those Module States
        mDrive.setModuleStates(targetSwerveModuleStates);

        

    }

    private void stop() {

        // Set Speeds into Swerve System
        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(0, 0, 0);

        // Call Autonomous Chasis Speed 
        var targetSwerveModuleStates = mDrive.getSwerveKinematics().toSwerveModuleStates(calculatedSpeeds);

        // Set Swerve to those Module States
        mDrive.setModuleStates(targetSwerveModuleStates);
    }



}
