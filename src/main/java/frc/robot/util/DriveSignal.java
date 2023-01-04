package frc.robot.util;

public class DriveSignal {
  private double mLeftMotor;
  private double mRightMotor;
  private final boolean mBrakeMode;

  public DriveSignal(double left, double right) {
    this(left, right, false);
  }

  public DriveSignal(double left, double right, boolean brakeMode) {
    mLeftMotor = left;
    mRightMotor = right;
    mBrakeMode = brakeMode;
  }

  public static DriveSignal fromControls(double throttle, double turn) {
    return new DriveSignal(throttle - turn, throttle + turn);
  }

  public static final DriveSignal NEUTRAL = new DriveSignal(0, 0);
  public static final DriveSignal BRAKE = new DriveSignal(0, 0, true);

  public void PrintLog() {
   // System.out.println("Left Output: " + mLeftMotor);
  //System.out.println("Right Output: " + mRightMotor);
  }

  public double getLeft() {
    return mLeftMotor;
  }

  public double getRight() {
    return mRightMotor;
  }

  public void setLeft(double value){
    mLeftMotor = value;
  }

  public void setRight(double value){
    mRightMotor = value;
  }

  public boolean getBrakeMode() {
    return mBrakeMode;
  }

  @Override
  public String toString() {
    return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
  }
}
