package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;

/**
 * Stores the Initial Drive Position Action
 * Useful for generating a backtracked position
 */
public class StoreDrivePositionAction implements Action {
  private final Drive mDrive  = Drive.getInstance();
  private boolean mIsComplete;

  public StoreDrivePositionAction() {
  }

  @Override
  public void start() {
    System.out.println("Storing Current Drive Action");
    mDrive.storeCurrentPose();
    mIsComplete = true;
  }

  @Override
  public void update() {

  }

  @Override
  public boolean isFinished() {
    return mIsComplete;
  }

  @Override
  public void done() {}
}
