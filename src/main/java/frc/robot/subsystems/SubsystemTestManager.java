package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

public class SubsystemTestManager {
    private static SubsystemTestManager mInstance;
  // Subsystems
  //private final Drive mDrive = Drive.getInstance();
  private final Arm mArm = Arm.getInstance();
  private final Grasper mGrasper = Grasper.getInstance();

  //timer
  private final Timer testTimer;

  //"is done" booleans for each tested thing
  private boolean shoulderCheck = false;
  private boolean extensionCheck = false;
  private boolean grasperCheck = false;

  //is done 
  public boolean isRunning = false;


  public static synchronized SubsystemTestManager getInstance() {
    if (mInstance == null) {
      mInstance = new SubsystemTestManager();
    }
    return mInstance;
  }


  private SubsystemTestManager(){
    testTimer = new Timer();
  }

  public void testInit(){
    if(!isRunning){
      testStart();
    }else{
      testUpdate();

      if(testUpdate() == true){
        testEnd();
      }
    }
  }

  public void testStart(){
    testTimer.start();
    mArm.setExtensionJog(.2);
    mArm.setShoulderJog(.2);
    mGrasper.setGrasperWheelIntake();
    shoulderCheck = false;
    extensionCheck = false;
    isRunning = true;

    mArm.TestExtensionMotor.setExpectedEncoderValue(0, 0);
    mArm.TestExtensionMotor.setExpectedCurrentValue(0, 0);

    mArm.TestShoulderMotor.setExpectedEncoderValue(0, 0);
    mArm.TestShoulderMotor.setExpectedCurrentValue(0, 0);

    mGrasper.TestGrasperMotor.setExpectedEncoderValue(0, 0);
    mGrasper.TestGrasperMotor.setExpectedCurrentValue(0, 0);
  }

  public boolean testUpdate(){
    mArm.TestShoulderMotor.setEncoderValue();
    mArm.TestExtensionMotor.setCurrentValue();
    mGrasper.TestGrasperMotor.setEncoderValue();

    mArm.TestShoulderMotor.setCurrentValue();
    mArm.TestExtensionMotor.setCurrentValue();
    mGrasper.TestGrasperMotor.setCurrentValue();

    if(testTimer.hasElapsed(2)){
      shoulderCheck = mArm.TestShoulderMotor.getEncoderStatus() && mArm.TestShoulderMotor.getCurrentStatus();
      extensionCheck = mArm.TestExtensionMotor.getEncoderStatus() && mArm.TestExtensionMotor.getCurrentStatus();
      grasperCheck = mGrasper.TestGrasperMotor.getEncoderStatus() && mGrasper.TestGrasperMotor.getCurrentStatus();
      testTimer.stop();
      return true;
    }
    return false;
  }

  public void testEnd(){
    testTimer.reset();
    mArm.setShoulderJog(0);
    mArm.setExtensionJog(0);
    mGrasper.cancelGrasperWheelIntake();
    isRunning = false;
    getStatus();
  }

  public void getStatus(){
    if(shoulderCheck){
      System.out.println("shoulder is good");
    }else{
      System.out.println("shoulder is broken");
    }
    if(extensionCheck){
      System.out.println("extension is good");
    }else{
      System.out.println("extension is broken");
    }
    if(grasperCheck){
      System.out.println("grasper is good");
    }else{
      System.out.println("grasper is broken");
    }
  }
    
}
