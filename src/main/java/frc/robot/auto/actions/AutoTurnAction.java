package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.*;
//import frc.robot.vision.*;
import edu.wpi.first.wpilibj.PowerDistribution;
/**
 * Uses Vision to turn to the ball
 */
public class AutoTurnAction implements Action {
  private Drive mDrive = Drive.getInstance();
  //private VisionManager mVisionManager = VisionManager.getInstance();
  private boolean mComplete;
  private double mNicksHelpingHand;
  private double mDeadband = 3;

  public AutoTurnAction(){
    mComplete = false;
    mNicksHelpingHand = 0;
  }
  public AutoTurnAction(double nicksHelpingHand){
    mComplete = false;
    mNicksHelpingHand = nicksHelpingHand;
    
  }

  public AutoTurnAction(double nicksHelpingHand, double db){
    mComplete = false;
    mNicksHelpingHand = nicksHelpingHand;
    mDeadband = db;
  }

  @Override
  public void start() {

  }

  @Override
  public void update(){
    System.out.println("in update loop for auto turn action.");
    /*
    TargetInfo ti = mVisionManager.getSelectedTarget(Constants.Vision.kAllowedSecondsThreshold);
    double errorAngle = 0;
    if(ti != null && ti.isValid()){
        // has valid error
        errorAngle = ti.getErrorAngle() - mNicksHelpingHand;
        System.out.println("error angle "+ errorAngle);
    }
    else {
      System.out.println("we didn't get anything");
    }
    // turn in place to errorAngle
    mDrive.driveErrorAngle(0, errorAngle);
    if(Math.abs(errorAngle - mNicksHelpingHand) <= mDeadband) {
      mComplete = true;
      System.out.println("Done!");
    }
    */
  }

  @Override
  public boolean isFinished() {
      return mComplete;
  }

  @Override
  public void done() {
    // stop driving
    mDrive.setDrive(0, 0, false);
  }
}