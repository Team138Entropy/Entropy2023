package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.util.CrashTrackingRunnable;
import frc.robot.util.Logger;



/**
 * This class selects, runs, and (if necessary) stops a specified autonomous mode.
 */
public class AutoModeExecutor {
    private static AutoModeExecutor mInstance = null;

    private AutoModeBase mAutoMode = null;
    private Thread mThread = null;

    public static AutoModeExecutor getInstance() {
        if (mInstance == null) {
            mInstance = new AutoModeExecutor();
        }

        return mInstance;
    }

    public AutoModeExecutor(){

        // Force TrajectoryLibrary to Load
        TrajectoryLibrary.getInstance();        
    }

    public void setAutoMode(AutoModeBase new_auto_mode) {
        mAutoMode = new_auto_mode;
        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                if (mAutoMode != null) {
                    System.out.println("AutoModeExecutor::runCrashTracked");
                    mAutoMode.run();
                }
            }
        });
        mThread.setPriority(6);
    }

    public void start() {
        Logger.log("AutoModeExecutor", "Start");
        if (mThread != null) {
            mThread.start();
        }
    }

    public boolean isStarted() {
        return mAutoMode != null && mAutoMode.isActive() && mThread != null && mThread.isAlive();
    }

    public void reset() {
        Logger.log("AutoModeExecutor", "Reset");
        if (isStarted()) {
            System.out.println("AutoModeExecutor::Reset::Stop!");
            stop();
        }

        mAutoMode = null;
    }

    public void stop() {
        Logger.log("AutoModeExecutor", "Stop");
        if (mAutoMode != null) {
            mAutoMode.stop();
        }
        
        mThread = null;
    }

    public AutoModeBase getAutoMode() {
        return mAutoMode;
    }

    public boolean isInterrupted() {
        Logger.log("AutoModeExecutor", "isInterrupted");
        if (mAutoMode == null) {
            return false;
        }
        return mAutoMode.getIsInterrupted();
    }

    public void interrupt() {
        if (mAutoMode == null) {
            return;
        }
        mAutoMode.interrupt();
    }

    public void resume() {
        System.out.println("AutoModeExecutor::resume");
        if (mAutoMode == null) {
            return;
        }
        mAutoMode.resume();
    }

    public boolean hasAutoMode()
    {
        return (null != mAutoMode);
    }

    public void updateSmartDashboard()
    {
        final String key = "AutoModeExecutor/";
        SmartDashboard.putBoolean(key + "HasAutoMode", hasAutoMode());
        if(hasAutoMode()) mAutoMode.updateSmartDashboard(key);
    }
}