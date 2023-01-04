package frc.robot.auto.actions;

import frc.robot.auto.TrajectoryFollower;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * Drives the Trajectory using the Trajectory Follower
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DriveTrajectoryAction implements Action {
    private TrajectoryFollower mTrajectoryFollower = TrajectoryFollower.getInstance();
    private Trajectory mTrajectory;
    private boolean mStopWhenDone;

    public DriveTrajectoryAction(Trajectory t, boolean stopWhenDone) {
        mTrajectory = t;
        mStopWhenDone = stopWhenDone;
    }

    public DriveTrajectoryAction(Trajectory p) {
        this(p, true);
    }

    @Override
    public void start() {
        System.out.println("DriveTrajectoryAction::Start");

        // set Trajectory into TrajectoryFollower
        mTrajectoryFollower.setTrajectory(mTrajectory);

        // perform intailization of trajectory follower
        mTrajectoryFollower.Start();
    }

    @Override
    public void update() {
        // update trajectory
        mTrajectoryFollower.Update();
    }

    // if trajectory is done
    @Override
    public boolean isFinished() {
        return mTrajectoryFollower.isComplete();
    }

    @Override
    public void done() {
        System.out.println("DriveTrajectoryAction::Done");
        if (mStopWhenDone) {
            mTrajectoryFollower.StopDrive();
        }
    }
}