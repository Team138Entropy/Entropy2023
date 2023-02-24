package frc.robot.auto.actions;

import frc.robot.auto.TrajectoryFollower;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Robot;
import frc.robot.Enums.SwerveCardinal;
import frc.robot.subsystems.Drive;
import frc.robot.vision.chargingStationAutoPilot;

/**
 * Drives the Trajectory using the Trajectory Follower
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
// https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
public class ChargingStationAction implements Action {
    private Drive mDrive = Drive.getInstance();
    private chargingStationAutoPilot mChargingStationAutoPilot = chargingStationAutoPilot.getInstance();

    public ChargingStationAction() {
       
    }

    @Override
    public void start() {
    
    }

    @Override
    public void update() {
        mChargingStationAutoPilot.update(false, false);
    }


    // if trajectory is done
    @Override
    public boolean isFinished() {
        return mChargingStationAutoPilot.getDone();
    }

    @Override
    public void done() {
        mDrive.setSwerveDrive(new Translation2d(0,0), 0, true, true, false);
    }
}