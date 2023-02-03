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


  //placeholder expected encoder value
  private final double expectedShoulderValue = 100;
  private final double expectedExtensionValue = 100;


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
    boolean isDone = false;
    do{
      testStart();
      

    }while(isDone == false);
  }

  public void testStart(){
    testTimer.start();
    mArm.setExtensionJog(.2);
    mArm.setShoulderJog(.2);
    mGrasper.setGrasperWheelIntake();
    shoulderCheck = false;
    extensionCheck = false;
  }

  public boolean testUpdate(){
    if(testTimer.hasElapsed(2)){
        if(mArm.getArmAngle() > expectedShoulderValue){
            shoulderCheck = true;
        }
        if(mArm.getArmExtension() > expectedExtensionValue){
            extensionCheck = true;
        }
        testTimer.stop();
        return true;
    }
    return false;
  }

  public boolean testEnd(){
    testTimer.reset();
    mArm.setShoulderJog(0);
    mArm.setExtensionJog(0);
    mGrasper.cancelGrasperWheelIntake();
    return true;
  }

  public void getStatus(){
    if(shoulderCheck && extensionCheck){
        System.out.println("everything is good");
    }else if(shoulderCheck == false && extensionCheck){
        System.out.println("The extension is good but the shoulder is broken");
    }else if(extensionCheck == false && shoulderCheck){
        System.out.println("The shoulder is good but the extension is broken");
    }else{
        System.out.println("Both the shoulder and the extension is broken");
    }
  }
    
}
