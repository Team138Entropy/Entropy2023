package frc.robot.auto.actions;

import frc.robot.subsystems.Grasper;

public class GrasperIntakeAction implements Action{
   private final Grasper mGrasper = Grasper.getInstance();
  
    public GrasperIntakeAction(){

    }
  
    @Override
    public void start() {

    }
  
    @Override
    public void update(){
      mGrasper.setGrasperOpen();
      mGrasper.update();
    }
  
    @Override
    public boolean isFinished() {
      return mGrasper.getBeamSensorBroken();
    }
  
    @Override
    public void done() {
      mGrasper.setGrasperClosed();
    }
}