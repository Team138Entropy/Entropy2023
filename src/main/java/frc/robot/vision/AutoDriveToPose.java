package frc.robot.vision;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Automatically Drive to a Pose2d
//      https://github.com/STMARobotics/swerve-test/blob/main/src/main/java/frc/robot/commands/ChaseTagCommand.java
//      https://www.chiefdelphi.com/t/how-do-you-typically-swerve-drive-to-poses-midgame/422862/4
//      https://github.com/FRCTeam2910/2021CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2020/commands/DriveToLoadingStationCommand.java

public class AutoDriveToPose {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = 
                    new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = 
                    new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   
                    new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    public AutoDriveToPose()
    {

    }                

    public void initialize()
    {

    }

}