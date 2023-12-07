package frc.robot.auto;

import frc.robot.subsystems.Drive;
import edu.wpi.first.math.trajectory.*;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

public class TrajectoryGeneratorHelper {
    private static TrajectoryConfig mTrajectoryConfig = null;

    // Initialize Trajectory Config
    public static void initTrajectoryConfig(){
        if(mTrajectoryConfig == null){
         // VelocityMetersPerSec, AccelerationMetersPerSec2
         mTrajectoryConfig = new TrajectoryConfig(4, 3.7);
         mTrajectoryConfig.setKinematics(Drive.getInstance().getKinematics());
        }
    }

    public static Trajectory generateTrajectory(Pose2d StartPose, Pose2d EndPose){
        // Generate Trajectory Config
        initTrajectoryConfig();

        // return computed trajectory
        return TrajectoryGenerator.generateTrajectory(StartPose, new ArrayList<Translation2d>(), 
                                                                    EndPose, mTrajectoryConfig); 
    }

    // Generate Example Trajectories
    // This is to get the JIT loaded to speed up dynamic trajectory
    public static void generateExampleTrajectories(){
        try {
            for(int i = 0; i < 10; i++){
                Trajectory traj = generateTrajectory(new Pose2d(10, 12, new Rotation2d(12)), 
                    new Pose2d(10, 12, new Rotation2d(12))
                );
                Trajectory traj1 = generateTrajectory(new Pose2d(10, 12, new Rotation2d(12)), 
                    new Pose2d(100, 100, new Rotation2d(100))
                );
                Trajectory traj2 = generateTrajectory(new Pose2d(10, 12, new Rotation2d(12)), 
                    new Pose2d(20, 20, new Rotation2d(20))
                );
                Trajectory traj3 = generateTrajectory(new Pose2d(10, 12, new Rotation2d(12)), 
                    new Pose2d(0, 0, new Rotation2d(0))
                );
                Trajectory traj4 = generateTrajectory(new Pose2d(10, 12, new Rotation2d(12)), 
                    new Pose2d(29, 20, new Rotation2d(0))
                );
                Trajectory traj5 = generateTrajectory(new Pose2d(10, 12, new Rotation2d(12)), 
                new Pose2d(29, 20, new Rotation2d(0))
            );
                Trajectory traj6 = generateTrajectory(new Pose2d(10, 12, new Rotation2d(12)), 
                new Pose2d(29, 20, new Rotation2d(0))
            );
            }
        }
        catch (Exception e) {
            //System.out.println();
        }
        
    }

    // getStraightTrajectory
    // intended to drive straight line
    // Units are supposed to be in meters but might not be completely true to meters
    public static Trajectory getStraightTrajectory(double MeterUnits){
        return generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
                 new Pose2d(MeterUnits, 0, new Rotation2d(0)));
    }
    public static Trajectory getStraightTrajectory2(double MeterUnits){
        //return generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                 //new Pose2d(MeterUnits, -0.9, Rotation2d.fromDegrees(0)));
                 ArrayList<Translation2d> Translations = new ArrayList<>();
                 //Translations.add(new Translation2d(.6, 0));
                 return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Translations, 
                 new Pose2d(MeterUnits, -0.9, Rotation2d.fromDegrees(2)), mTrajectoryConfig); 
    }


    // x (straight of robot)
    // left and right of robot
    public static Trajectory getLTrajectory(){
        // Generate Trajectory Config
        initTrajectoryConfig();

        // Start and End Pose
        Pose2d StartPose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));

        // meters forward , meters left/right
        Pose2d EndPose = new Pose2d(.8, -2, new Rotation2d().fromDegrees(90));

        // return computed trajectory
        return TrajectoryGenerator.generateTrajectory(
                StartPose, 
                new ArrayList<Translation2d>(), 
                EndPose, 
                mTrajectoryConfig); 
    }

    public static Trajectory testTrajectory(){
        return generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
                 new Pose2d(.3, 1, new Rotation2d(.69)));
    }
}
