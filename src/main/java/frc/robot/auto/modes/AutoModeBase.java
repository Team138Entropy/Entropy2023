package frc.robot.auto.modes;

import frc.robot.FieldConstants;
import frc.robot.Enums.SwerveRotation;
import frc.robot.Enums.TargetedPositions;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.NoopAction;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private double mEstimatedDuration = 0;

    // Starting Pose 
    public Optional<Pose2d> mStartingPose = Optional.empty();   
    
    // Starting Position 
    public Optional<TargetedPositions> mStartingPosition = Optional.empty();
    public Optional<SwerveRotation> mStartingRotation = Optional.empty();

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

    // Pose to Drive To
    public void driveToPose(Pose2d targetPose)
    {

    }

    // Pose to Drive To with Limited Speed
    public void driveToPose(Pose2d targetPose, double value)
    {

    }

    public void driveToPose(Pose2d targetPose, Action otherAction)
    {

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

    // Current Action Index
    public int getCurrentAction()
    {
        return mCurrentAction;
    }

    // Get Action Count
    public int getActionCount()
    {
        return mAutoActions.size();
    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }

    // This adds a starting Pose Action
    public void setStartingPose(Pose2d pose)
    {
        mStartingPose = Optional.of(pose);
    }

    // Starting Position (if applicable)
    public void setStartingPosition(
        TargetedPositions pos, SwerveRotation rot
    )
    {
        mStartingPosition = Optional.of(pos);
        mStartingRotation = Optional.of(rot);
    }

    // Has a Starting Position been set?
    public boolean hasStartingPosition()
    {
        return (!mStartingPosition.isEmpty() &&
            !mStartingRotation.isEmpty()
        );
    }

    // Calculate the Starting Position
    // Alliance Color is needed to know this
    public void calculateStartingPosition(Alliance allianceColor) 
    {
        if(hasStartingPosition())
        {
            // Has a Starting Position, find the pose for it
            Pose2d pose = FieldConstants.Auto.getStartingPose(
                mStartingPosition.get(), 
                mStartingRotation.get(),
                allianceColor
            );
            mStartingPose = Optional.of(pose);
        }
    }

    // Has a Starting Pose
    public boolean hasStartingPose()
    {
        return !mStartingPose.isEmpty();
    }

    // Get a Starting Pose (otherwise returns empty)
    public Pose2d getStartingPose()
    {
        return hasStartingPose() ? mStartingPose.get() : new Pose2d();
    }

    // Set Alliance Color
    public void setAllianceColor(Alliance allianceColor)
    {
        calculateStartingPosition(allianceColor);
    }

    // Incriment Estimated Duration
    public void incrimentDuration(double seconds)
    {
        mEstimatedDuration += seconds;
    }

    // Get Estimated Duration 
    public double getEstimatedDuration()
    {
        return mEstimatedDuration;
    }

    public void updateSmartDashboard(String key)
    {
        SmartDashboard.putBoolean(key + "Running", isActive());
        SmartDashboard.putNumber(key + "Current Action", getCurrentAction() + 1);
        SmartDashboard.putNumber(key + "Total Actions", getActionCount());
        SmartDashboard.putNumber(key + "Estimated Duration", getEstimatedDuration());

    }
}