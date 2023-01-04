package frc.robot.auto.actions;

import frc.robot.auto.TrajectoryFollower;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

/**
 * Drives the Trajectory using the Trajectory Follower
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
// https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
public class TurnInPlaceAction implements Action {
    private boolean mComplete;
    private double mDegrees;
    private double mGyroStart;
    private double mError;
    private Drive mDrive = Drive.getInstance();

    public TurnInPlaceAction(double degrees) {
        mComplete = false;
        mDegrees = degrees;

    }

    @Override
    public void start() {
        System.out.println("TurnInPlaceAction - Target Degrees" + mDegrees);
        mGyroStart = mDrive.getGyro().getAngle();
    }

    @Override
    public void update() {
        updateError();
        System.out.println("TurnInPlaceAction - Update - Error: ");
        System.out.println(mError);

        // turn drive by error angle
        mDrive.driveErrorAngle(0, mError);
        if (Math.abs(mError) <= 5.5) {
            System.out.println("Within allowed error");
            // within degrees
            mComplete = true;
        }
    }

    // Update the Error
    private void updateError() {

        mError = (mDegrees - mGyroStart ) - mDrive.getGyro().getAngle();
    }

    // if trajectory is done
    @Override
    public boolean isFinished() {
        return mComplete;
    }

    @Override
    public void done() {
        System.out.println("Action: Turn in Place Complete");
        mDrive.setPercentOutputDrive(0, 0);
    }
}