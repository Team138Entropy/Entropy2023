package frc.robot.auto.actions;

import frc.robot.subsystems.Grasper;

public class GrasperEjectAction implements Action{
   private final Grasper mGrasper = Grasper.getInstance();
   private final int mRunTime = 10;
   private int mLoopCount;
  
    public GrasperEjectAction(){

    }
  
    @Override
    public void start() {
      mLoopCount = 0;
    }
  
    @Override
    public void update(){
      mGrasper.setGrasperClosed();
      
      mLoopCount++;
    }
  
    @Override
    public boolean isFinished() {
      return mLoopCount > mRunTime;
    }
  
    @Override
    public void done() {
      mGrasper.setGrasperClosed();
    }
}