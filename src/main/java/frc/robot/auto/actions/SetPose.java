package frc.robot.auto.actions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.Drive;

// Set Pose
// Used to Update the Odometery of where the robot is on the field
// Should really only be used at the start of an auto routine
public class SetPose implements Action {
    private Drive mDrive = Drive.getInstance();
    private Pose2d mTargetPose;

    public SetPose(Pose2d targetPose)
    {
        mTargetPose = targetPose;
    }
    
    @Override
    public void start()
    {
        // Set Robots Odometrey to this point
        mDrive.resetOdometry(mTargetPose);
    }

    @Override 
    public void update() {
    }

    @Override
    public void done() {
        
    }

    @Override 
    public boolean isFinished() {
        return true;
    }
}
