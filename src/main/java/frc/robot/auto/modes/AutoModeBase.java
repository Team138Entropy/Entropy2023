package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.NoopAction;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    protected final double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;
    public List<Action> mAutoActions = new ArrayList<>();
    public int mCurrentAction = 0;
    public boolean mHasStartedAction = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        mActive = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            System.out.println("Auto mode ended early");
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void reset()
    {
        mCurrentAction = 0;
        mHasStartedAction = false;
    }

    public boolean isDone()
    {
        return (mCurrentAction >= mAutoActions.size());
    }

    public void runner(){
        if(!isDone()){
            if(!mHasStartedAction){
                mAutoActions.get(mCurrentAction).start();
                mHasStartedAction = true;
            }
            else if(!mAutoActions.get(mCurrentAction).isFinished()){
                mAutoActions.get(mCurrentAction).update();
            }
            else{
                mAutoActions.get(mCurrentAction).done();
                mCurrentAction++;
                mHasStartedAction = false;
            }
        }
    }

    public void addAction(Action a)
    {
        registerAction(a);
    }

    public void registerAction(Action a)
    {
        mAutoActions.add(a);
    }

    public void done() {
        System.out.println("Auto mode done");
    }

    public void stop() {
        System.out.println("Auto mode stop");
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void waitForDriverConfirm() throws AutoModeEndedException {
        if (!mIsInterrupted) {
            interrupt();
        }
        runAction(new NoopAction());
    }

    public void interrupt() {
        System.out.println("** Auto mode interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        System.out.println("** Auto mode resumed!");
        mIsInterrupted = false;
    }

    //
    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 500.0);

        // Wait for interrupt state to clear
        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                System.out.println("AutoModeBase::IsActiveWIthThrow::Sleep");
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                System.out.println("AutoModeBase::Exception1");
                e.printStackTrace();
            }
        }

        System.out.println("AutoModeBase::Start");
        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            System.out.println("AutoModeBase::Update");

            action.update();

            try {
        System.out.println("AutoModeBase::Sleep");

                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
        System.out.println("AutoModeBase::Exception2");

                e.printStackTrace();
            }
        }
        System.out.println("AutoModeBase::Done");

        action.done();

    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }
}