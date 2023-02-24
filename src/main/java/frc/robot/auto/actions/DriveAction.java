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

/**
 * Drives the Trajectory using the Trajectory Follower
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
// https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
public class DriveAction implements Action {
    private Drive mDrive = Drive.getInstance();
    private Translation2d mTranslation;
    private double mSeconds;
    private Timer mTimer =new Timer();

    public DriveAction(Translation2d translation, double seconds) {
        mTranslation = translation;
        mSeconds = seconds;

    }

    @Override
    public void start() {
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void update() {
        mDrive.setSwerveDrive(mTranslation, 0, true, true, false);
    }


    // if trajectory is done
    @Override
    public boolean isFinished() {
        return mTimer.hasElapsed(mSeconds);
    }

    @Override
    public void done() {
        mDrive.setSwerveDrive(new Translation2d(0,0), 0, true, true, false);
    }
}