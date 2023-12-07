package frc.robot.auto.actions;

import frc.robot.auto.TrajectoryFollower;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Robot;
import frc.robot.Enums.SwerveCardinal;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;

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
    private SwerveCardinal mSnap;

    public TurnInPlaceAction(SwerveCardinal snap) {
        mComplete = false;
        mSnap = snap;

    }

    @Override
    public void start() {
        checkSnapCG();
        mDrive.startSnap(mSnap.degrees);
    }

    @Override
    public void update() {
        mDrive.setSwerveDrive(new Translation2d(), 0, true, true, false);
        System.out.println("Action: Turn in Place running");
    }


    // if trajectory is done
    @Override
    public boolean isFinished() {
        return !mDrive.isSnapping();
    }

    @Override
    public void done() {
        System.out.println("Action: Turn in Place Complete");
        mDrive.setSwerveDrive(new Translation2d(), 0, true, true, false);
    }

    // Configure
    private void checkSnapCG()
    {
        if(Superstructure.getInstance().isCGCompromised())
        {
            mDrive.setCGUnsafeSnap();
        }else {
            mDrive.setNormalSnap();
        }
    }
}