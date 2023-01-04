package frc.robot.util;

import java.util.Vector;
import java.util.concurrent.Callable;
import edu.wpi.first.wpilibj.Timer;

class Stage {
    public final Callable<Boolean> mStageWorkerFunction;
    public final Callable<Boolean> mStageCompleteFunction;
    public final String mName;
    public final boolean mRequireUserToStart;
    public final double mSecondsDelay;

    public Stage(String Name, Callable<Boolean> workerFunc, Callable<Boolean> completeFunc){
        mName = Name;
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = false;
        mSecondsDelay = 0;
    }

    public Stage(String Name, Callable<Boolean> workerFunc, Callable<Boolean> completeFunc, boolean reqStart){
        mName = Name;
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = reqStart;
        mSecondsDelay = 0;
    }

    public Stage(String Name, Callable<Boolean> workerFunc, Callable<Boolean> completeFunc, boolean reqStart, double SecondsDelay){
        mName = Name;
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = reqStart;
        mSecondsDelay = SecondsDelay;
    }

    public Stage(Callable<Boolean> workerFunc, Callable<Boolean> completeFunc){
        mName = "";
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = false;
        mSecondsDelay = 0;
    }

    public Stage(Callable<Boolean> workerFunc, Callable<Boolean> completeFunc, boolean reqStart){
        mName = "";
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = reqStart;
        mSecondsDelay = 0;
    }

    public boolean hasName(){
        return mName.length() > 0;
    }
};

/* Executes a Stage, Checks conditions and then goes to the next stage
 * 
 */
public class StageExecutor {
    
    private int mCurrentStage;
    private int mCurrentStageLoopCount;
    private String mCurrentStageName;
    private boolean mVerboseMode;
    private boolean mRunning;
    private boolean mNeedUserInputToStartStage;
    private boolean mStartBlessed;
    private Vector<Stage> mStages;
    private Timer mTimer;
    private Timer mCurrentStageTimer;

    public StageExecutor(){
        mTimer = null;
        mCurrentStageTimer = null;
        mCurrentStage = 0;
        mCurrentStageLoopCount = 0;
        mVerboseMode = false;
        mRunning = false;
        mNeedUserInputToStartStage = false;
        mStartBlessed = false;
        mCurrentStageName = "";
        mStages = new Vector<Stage>();
    }

    // Adds a New Stage to be Executed
    // StageWorkerFunction is called every loop while in a stage
    public synchronized void registerStage(Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction){
        mStages.add(new Stage(stageWorkerFunction, stageCompleteFunction));
    }

    public synchronized void registerStage(Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction, boolean ReqUserInputStart){
        mStages.add(new Stage(stageWorkerFunction, stageCompleteFunction, ReqUserInputStart));
    }

    // Adds a New Stage to be Executed
    public synchronized void registerStage(String Name, Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction){
        mStages.add(new Stage(Name, stageWorkerFunction, stageCompleteFunction));
    }

    public synchronized void registerStage(String Name, Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction, boolean ReqUserInputStart){
        mStages.add(new Stage(Name, stageWorkerFunction, stageCompleteFunction, ReqUserInputStart));
    }

    public synchronized void registerStage(String Name, Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction, boolean ReqUserInputStart, double seconds){
        mStages.add(new Stage(Name, stageWorkerFunction, stageCompleteFunction, ReqUserInputStart, seconds));
    }

    // Updates the Stage Executor
    public synchronized boolean update(){
        return update(false);
    }

    // Updates the Stage Executor
    // Allows a User Input to Approve Next Stage (if Stage Requires)
    // returns true when complete
    public synchronized boolean update(boolean userInput){
        boolean result = false;
        if(mCurrentStage < mStages.size()){
            // still at a valid stage
            Stage currentStage = mStages.get(mCurrentStage);
            mCurrentStageLoopCount++;
            mCurrentStageName = currentStage.mName;
            
            // first loop, store that we need to get user input
            if(mCurrentStageLoopCount == 1){
                mNeedUserInputToStartStage = currentStage.mRequireUserToStart;
            }
            if(mVerboseMode) System.out.println("Executing Stage: " + (mCurrentStage + 1));
            try {
                // check if we need blessing to start
                if(mNeedUserInputToStartStage && !mStartBlessed){
                    if(mVerboseMode) System.out.println("Waiting for User Input to Start Stage: " + (mCurrentStage + 1));
                    if(userInput){
                        // user says start
                        mStartBlessed = true;
                        mNeedUserInputToStartStage = false;
                    }else{
                        if(mVerboseMode) System.out.println("No Start Blessing - Next Loop!");
                        // no start blessing, go to next loop
                        return false;
                    }
                }

                // create stage executing climber and start (if not done prior)
                if(mCurrentStageTimer == null){
                    mCurrentStageTimer = new Timer();
                    mCurrentStageTimer.start();
                }

                // Executing Stage
                currentStage.mStageWorkerFunction.call();
            }catch(Exception e){
                // Exception Caught
                System.out.println("StageExecutor Worker Exception!");
                System.out.println(e.getMessage());
            }
            
            
            // Check if Stage is Complete
            try{
                if(currentStage.mStageCompleteFunction.call()){
                    // Complete Stage is Hit
                    if(mTimer == null){
                        mTimer = new Timer();
                        mTimer.start();
                    }

                    // verify post delay is complete!
                    if(mTimer.hasElapsed(currentStage.mSecondsDelay)){
                        // Move to Next Stage
                        if(mVerboseMode) System.out.println("Stage " + (mCurrentStage + 1) + " Complete!");
                        mCurrentStage++;
                        mCurrentStageLoopCount = 0; //clear out loop count
                        mStartBlessed = false;
                        mNeedUserInputToStartStage = false;
                        mTimer.stop();
                        mTimer = null;
                        mCurrentStageTimer = null;
                    }else{
                        // Still Waiting for stage to finish
                        if(mVerboseMode) System.out.println("Waiting for Stage " + (mCurrentStage + 1) + " Post Delay to Complete!");
                    }
                }else{
                    // Stage Not Complete
                    if(mVerboseMode) System.out.println("Stage " + (mCurrentStage + 1) + " Not Complete!");
                }
            }catch(Exception e){
                // exception
                System.out.println("StageExecutor Complete Exception!");
                System.out.println(e.getMessage());                
            }    
            
        }else{
            // Stages Complete
            result = true;
            if(mVerboseMode) System.out.println("Stages Complete!");
        }
        return result;
    }

    public synchronized void reset(){
        if(mVerboseMode) System.out.println("Reset StageExecutor");
        mCurrentStage = 0;
        mCurrentStageLoopCount = 0; //clear out loop count
        mStartBlessed = false;
        mNeedUserInputToStartStage = false;
        mTimer = null;
        mCurrentStageTimer = null;
    }

    public synchronized void resetToStage(int stage){
        stage -= 1; //convert stage to zero based
        if(stage >= 0 && stage < mStages.size()){
            reset();
            mCurrentStage = stage;
        }
    }

    public synchronized void setVerboseMode() { mVerboseMode = true;}
    public synchronized void disableVerboseMode() { mVerboseMode = false;}

    // Stages are 1 Based, there is no stage 0
    public synchronized int getCurrentStage(){
        return mCurrentStage + 1;
    }
    
    public synchronized String getCurrentStageName(){
        return mCurrentStageName;
    }

    public synchronized boolean isComplete(){
        return mCurrentStage >= mStages.size();
    }

    public synchronized boolean needUserInputToStart(){
        return mNeedUserInputToStartStage;
    }

    /**
     * 
     * @return Current Stage Execution Time in Seconds
     */
    public synchronized double getCurrentStageExecutionTime(){
        return mCurrentStageTimer == null ? 0 : mCurrentStageTimer.get();
    }
}
