package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {
    private final double mTimeToWait;
    private double mStartTime;
    private Timer mTimer;

    public WaitAction(double timeToWait) {
        mTimeToWait = timeToWait;
    }

    @Override
    public void start() {
        mTimer = new Timer();
        mTimer.start();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mTimer.hasElapsed(mTimeToWait);
    }

    @Override
    public void done() {}
}