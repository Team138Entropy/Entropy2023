package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Composite action, Runs all actions 1 at a time
 */
public class SequentialAction implements Action {
    private final ArrayList<Action> mActions;
    private final int mActionCount;
    private int mIndex;


    public SequentialAction(List<Action> actions) {
        mActions = new ArrayList<>(actions);
        mActionCount = mActions.size();
    }

    public SequentialAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public void start() {
        mIndex = 0;
        if(mIndex < mActionCount) mActions.get(mIndex).start();
    }

    @Override
    public void update() {
        if(!mActions.get(mIndex).isFinished())
        {
            // Action not finished, update
            mActions.get(mIndex).update();
        }
        else 
        {
            // Action finished, incriment
            ++mIndex;
        }
    }

    @Override
    public boolean isFinished() {
        // If beyond last action
        return (mIndex >= mActionCount);
    }

    @Override
    public void done() {
        //mActions.forEach(Action::done);
    }
}