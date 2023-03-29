package frc.robot;
import edu.wpi.first.math.geometry.*;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Enums.ArmTargets;
import frc.robot.util.Triplet;
import frc.robot.util.TuneableNumber;
import frc.robot.util.drivers.SwerveModuleConstants;
import frc.robot.util.physics.ArmConstraint;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;

/**
 * Constants
 * Values defined in this class will never change during robot operation
 */
public class Constants {
  // Used for tuneable numbers
  public static final boolean tuningMode = true;

  // Potential Targets
  public enum TargetType {
    CAMERA_1_RED_CARGO,
    CAMERA_1_BLUE_CARGO,
    CAMERA_2_RED_CARGO,
    CAMERA_2_BLUE_CARGO;
  }

  // Talon Can IDs
  // CAN IDs
  public static class Talons {
    public static final String auxCanBus = "canivore";
    public static class Drive {
      public static final int leftMaster = 4;
      public static final int leftSlave = 3;
      public static final int rightMaster = 2;
      public static final int rightSlave = 1;

      /* Swerve Module IDs */
      public static class SwerveModules {
        /* Module 0 */
        public static final int module0_DriveMotor = 1;
        public static final int module0_RotationMotor = 2;
        public static final int module0_Cancoder = 3;

        /* Module 1 */
        public static final int module1_DriveMotor = 5;
        public static final int module1_RotationMotor = 4;
        public static final int module1_Cancoder = 6;

        /* Module 2 */
        public static final int module2_DriveMotor = 7;
        public static final int module2_RotationMotor = 8;
        public static final int module2_Cancoder = 9;

        /* Module 3 */
        public static final int module3_DriveMotor = 10;
        public static final int module3_RotationMotor = 11;
        public static final int module3_Cancoder = 12;
      }

    }

    public static class Arm {
      public static final int ShoulderMasterId = 16;
      public static final int ShoulderSlaveId = 17;
      public static final int ExtensionId = 18;
    }

    public static class Grasper {
      public static final int IntakeMotor = 19;
      public static final int PCMId = 0;
    }

    public static class Sensors {
      public static final int pigeonCan = 25;
    }

    public static class PowerDistribution {
      public static final int pdpCan = 20;
    }
  }

  public static class Vision {
    public static final double diagonalView = Math.toRadians(170);
    public static final double horizontalAspect = 4;
    public static final double verticalAspect = 3;
    public static final double diagonalAspect = Math.hypot(horizontalAspect, verticalAspect);
    public static final double horizontalView =
      Math.atan(Math.tan(diagonalView / 2) * (horizontalAspect / diagonalView)) * 2;
    public static final double verticalView =
      Math.atan(Math.tan(diagonalView / 2) * (verticalAspect / diagonalView)) * 2;
    
    public static final Rotation2d kCameraHorizontalPlaneToLens =
      Rotation2d.fromDegrees(0);

    // Allowed Seconds Threshold
    public static final double kAllowedSecondsThreshold = 5; //seconds

    public static final double kAllowedTargetArea = 0.1; // How big must a target be
  }

  public static class AutoPilot {
    //tunable PID values
    public static TuneableNumber CSAutoPilotKP = new TuneableNumber("CSAutoPilotKP",2.5);
    public static TuneableNumber CSAutoPilotKI = new TuneableNumber("CSAutoPilotKI",0);
    public static TuneableNumber CSAutoPilotKD = new TuneableNumber("CSAutoPilotKD",0);
    public static TuneableNumber PitchAngleRateThreshold = new TuneableNumber("PitchAngleRateThreshold",5);
    public static TuneableNumber maxLevelSpeedLow = new TuneableNumber("maxLevelSpeedLow",-.4);
    public static TuneableNumber maxLevelSpeedHigh = new TuneableNumber("maxLevelSpeedHigh",.4);
    public static TuneableNumber ChargeStationDegreeThreshold = new TuneableNumber("ChargeStationDegreeThreshold",9);

  }

  // Subsystems
  public static class Drive {
    public static boolean enabled = true;

    public static int talonSensorTimeoutMs = 250;
  
    public static double maxSpeedWhenClimbing = .3;
  
    // TODO: determine if this is actually "DRIVE__FORWARD_ACCEL_RAMP_TIME_SECONDS", as it was
    // previously named
    public static double accelSpeed = 1;
    public static double brakeSpeed = 1;


    public static class Encoders {
      public static final double ticksPerWheelRotation = 22000.0;
      public static final double ticksPerMeters = ticksPerWheelRotation * RobotDimensions.RotationsPerMeter;

      // OLD VALUE
      // ticks = (19711 + 19582) / 2
      // distance in feet = 89.5/12
      // ticks per foot = ticks / feet
      private static final double compTicks = (19711.0 + 19582.0) / 2.0;
      private static final double compDistance = 89.5 / 12.0;
    

      public static double compTicksPerFoot = compTicks / compDistance;
      public static final double practiceTicksPerFoot = 1228.615;
    }

      public static class Auto {
        public static final double MaxVelocityMetersPerSecond = 1.8288; // m/s, from 6 ft/s
        public static final double MaxAccelerationMetersPerSecondSq = 1.2192; //m/s^2 from 4 ft/s^2
      }

      // Diameter of wheel & ticksPerRevolution used to convert meters into encoder ticks
      public static double drivetrainWheelDiameter = 0.1524; // 6 inches in meters
      public static double ticksPerRevolution = 4096;


    public static class AutoPID {
      public static final double p = 4;
      public static final double i = 0.00050;
      public static final double d = 0;
    }
  
    public static class AutoTurnPID {
      public static final double p = 0.2;
      public static final double i = 0;
      public static final double d = 0.015;
      public static final double acceptableError = 0.5;
    }

    public static final class QuickAdjustConstants {
      public static final double x_Kp = 3.0;
      public static final double x_Ki = 0.0;
      public static final double x_Kd = 0.0;

      public static final double y_Kp = 3.0;
      public static final double y_Ki = 0.0;
      public static final double y_Kd = 0.0;

      public static final double maxVelocity = 3;
      public static final double maxAcceleration = 2;

      public static final TrapezoidProfile.Constraints xConstraints =
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
      
      public static final TrapezoidProfile.Constraints yConstraints =
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    };

    public static final class SnapConstants_Normal {
      public static final double kP = 8.0; 
      public static final double kI = 0;
      public static final double kD = 0.0;
      public static final double kTimeout = 0.25;
      public static final double kEpsilon = 4.0;

      // Constraints for the profiled angle controller
      public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

      public static final double kAutoPilotMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
      public static final double kAutoPilotMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
              new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
              public static final TrapezoidProfile.Constraints kAutoPilotThetaControllerConstraints =
              new TrapezoidProfile.Constraints(kAutoPilotMaxAngularSpeedRadiansPerSecond, kAutoPilotMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class SnapConstants_CGUnsafe {
    public static final double kP = 5.0; 
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kTimeout = 0.25;
    public static final double kEpsilon = 4.0;

    // Constraints for the profiled angle controller
    public static final double kMaxAngularSpeedRadiansPerSecond = 1.4 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final double kAutoPilotMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kAutoPilotMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
            public static final TrapezoidProfile.Constraints kAutoPilotThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kAutoPilotMaxAngularSpeedRadiansPerSecond, kAutoPilotMaxAngularSpeedRadiansPerSecondSquared);
}

    /* Swerve Modules */
    // Each Angle Offset is a measured value from the CANCoder
    // More information detailed in the Swerve Module Class 
    public static class SwerveModules {
      public static class Module0 {
        public static double AngleOffset = 205.4;

        public static SwerveModuleConstants SwerveModuleConstants() {
          return new SwerveModuleConstants(
            "Front Left Module - Module 0",
            Talons.Drive.SwerveModules.module0_DriveMotor,
            Talons.Drive.SwerveModules.module0_RotationMotor,
            Talons.Drive.SwerveModules.module0_Cancoder,
            Talons.auxCanBus,
            AngleOffset
          );
        }
      }

      public static class Module1 {
        public static double AngleOffset = 223.33;

        public static SwerveModuleConstants SwerveModuleConstants() {
          return new SwerveModuleConstants(
            "Front Right Module - Module 1",
            Talons.Drive.SwerveModules.module1_DriveMotor,
            Talons.Drive.SwerveModules.module1_RotationMotor,
            Talons.Drive.SwerveModules.module1_Cancoder,
            Talons.auxCanBus,
            AngleOffset
          );
        }
      }

      public static class Module2 {
        public static double AngleOffset = 156.5;

        public static SwerveModuleConstants SwerveModuleConstants() {
          return new SwerveModuleConstants(
            "Back Left Module - Module 2",
            Talons.Drive.SwerveModules.module2_DriveMotor,
            Talons.Drive.SwerveModules.module2_RotationMotor,
            Talons.Drive.SwerveModules.module2_Cancoder,
            Talons.auxCanBus,
            AngleOffset
          );
        }
      }

      public static class Module3 {
        public static double AngleOffset = 37.08;

        public static SwerveModuleConstants SwerveModuleConstants() {
          return new SwerveModuleConstants(
            "Back Right Module - Module 3",
            Talons.Drive.SwerveModules.module3_DriveMotor,
            Talons.Drive.SwerveModules.module3_RotationMotor,
            Talons.Drive.SwerveModules.module3_Cancoder,
            Talons.auxCanBus,
            AngleOffset
          );
        }
      }
    }


  } // End Drive

  public static class Controllers {
    public static final boolean ignore = true;
  
    public static final double joystickDeadband = 0.15;
    public static final double triggerDeadband = 0.15;
  
    public static class Driver {
      public static final int port = 0;
      public static final String name = "Controller (Xbox One For Windows)";
    }
  
    public static class Operator {
      public static final int port = 1;
      public static final String name = "AIRFLO";
    }

    public static class Operator2 {
      public static final int port = 2;
      public static final String name = "AIRFLO";
    }
  }

  public static class RobotDimensions {
    // tested via trial & error, not measured
    // 22 is too low, 100 is too high
    public static final double driveWheelTrackWidthInches = 50;
    // Based on how this is used, I'm pretty sure this is a corrective factor
    public static final double trackScrubFactor = 1.0469745223;
  
    public static final double driveWheelDiameterInches = 3.938;
    public static final double driveWheelRadiusInches = driveWheelDiameterInches / 2.0;
    public static final double driveWheelTrackRadiusWidthMeters =
      driveWheelTrackWidthInches / 2.0 * 0.0254;

    // Temp used for Swerve System
    public static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(20.75);
  
    // Offsets from our center point
    public static final Pose2d turretToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d wheelsToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));

    // Wheel Dimensions
    private static final double WheelDiameter = 6.0; //inches
    private static final double WheelRadius = WheelDiameter/2.0; //inches
    private static final double WheelCircumference = 2 * Misc.pi * WheelRadius; //inches
    private static final double WheelCircumferenceMeters = WheelCircumference * Misc.inchesToMeters; //meters
    private static final double RotationsPerMeter = 1.0/WheelCircumferenceMeters; // meters (rotations per meter)
    public static final double tippingLimitXaxis = .25;


    // these aren't correct
  }

  public static class Shooter {
    public static final TuneableNumber shooterTestSpeed_Forward 
              = new TuneableNumber("shooter/TestSpeed_Forward", .4);
    public static final TuneableNumber shooterTestSpeed_Reverse 
              = new TuneableNumber("shooter/TestSpeed_reverse", -.4);
  }

  public static class Feeder {
    public static final TuneableNumber inputWheelTestSpeed_Forward 
              = new TuneableNumber("feeder/inputWheelTestSpeed_Forward", .4);
    public static final TuneableNumber inputWheelTestSpeed_Reverse 
              = new TuneableNumber("feeder/inputWheelTestSpeed_Reverse", -.4);

    public static final TuneableNumber feederTestSpeed_Forward
              = new TuneableNumber("feeder/feederTestSpeed_Forward", .3);
    public static final TuneableNumber feederTestSpeed_Reverse
              = new TuneableNumber("feeder/feederTestSpeed_Reverse", -.3);
  }

  public static class Arm {
    public static final double shoulderTicksPerRotation = 8192;
    public static final int shoulderStartPosition = 90;
    public static final int shoulderMinRotation = -40;
    public static final int shoulderMaxRotation = 220;

    public static final TuneableNumber jogSpeed = new TuneableNumber("armShoulderJogSpeed",.4);
                                                      
    public static final double MaxExtensionPosition = 245000.0;
    public static final double MinExtensionPosition = 0;

    // Arbitary Feedforwards
    //  Values should be the same but a second value is available in case 1 side behaves differently
    public static final double mArmRotationBacksideFF = 0; // This should be positive to get it to go up
    public static final double mArmRotationFrontsideFF = 0; // This should be negative to get it to go up

    // Overrides of the Arm Targets if Targetting Cube
    public static final Map<ArmTargets,ArmTargets> CubeArmTargetOverrides = new HashMap<ArmTargets, ArmTargets>();
    static {
      // Add Arm Overrides
      CubeArmTargetOverrides.put(Enums.ArmTargets.TOP_SCORING_FRONT, Enums.ArmTargets.TOP_SCORING_FRONT_CUBE);
      CubeArmTargetOverrides.put(Enums.ArmTargets.MID_SCORING_FRONT, Enums.ArmTargets.MID_SCORING_FRONT_CUBE);
      CubeArmTargetOverrides.put(Enums.ArmTargets.INTAKE_FRONT, Enums.ArmTargets.INTAKE_FRONT_CUBE);
      CubeArmTargetOverrides.put(Enums.ArmTargets.INTAKE_BACK, Enums.ArmTargets.INTAKE_BACK_CUBE);
      CubeArmTargetOverrides.put(Enums.ArmTargets.INTAKE_GROUND_FRONT, Enums.ArmTargets.INTAKE_GROUND_FRONT_CUBE);
    }

    //tunable PID values
    // Originally was 1 - 29 - .01 - 600
    public static TuneableNumber tunableArmKF_Prim = new TuneableNumber("tunableArmKF_Prim",0);
    public static TuneableNumber tunableArmKP_Prim = new TuneableNumber("tunableArmKP_Prim",1.8);
    public static TuneableNumber tunableArmKI_Prim = new TuneableNumber("tunableArmKI_Prim",0.00001);
    public static TuneableNumber tunableArmKD_Prim = new TuneableNumber("tunableArmKD_Prim",80);

    public static TuneableNumber tunableArmKF_Sec = new TuneableNumber("tunableArmKF_Sec",0);
    public static TuneableNumber tunableArmKP_Sec = new TuneableNumber("tunableArmKP_Sec",1.8);
    public static TuneableNumber tunableArmKI_Sec = new TuneableNumber("tunableArmKI_Sec",0.00001);
    public static TuneableNumber tunableArmKD_Sec = new TuneableNumber("tunableArmKD_Sec",80);

    // Distance Which the Arm Switches to Steady State Pid
    public static TuneableNumber tunableArmSteadyState = new TuneableNumber("tuneableArmSteadyState",5);


    public static TuneableNumber tunableArmVel = new TuneableNumber("tunableArmVel",1400);
    public static TuneableNumber tunableArmAccel = new TuneableNumber("tunableArmAccel",1400);
    public static TuneableNumber tuneableArmClosedLoopError = new TuneableNumber("tuneableArmClosedLoopError",25);

    // Constraints - A
    public static Vector<ArmConstraint> ArmConstraints;
    static {
      ArmConstraints = new Vector<ArmConstraint>();

      // must me pulled in on over the top rotations
      ArmConstraints.add(new ArmConstraint(70.0, ArmConstraint.ArmConstraintType.ExtensionMaximum, 0.0));
      ArmConstraints.add(new ArmConstraint(90.0, ArmConstraint.ArmConstraintType.ExtensionMaximum, 0.0));
      ArmConstraints.add(new ArmConstraint(110.0, ArmConstraint.ArmConstraintType.ExtensionMaximum, 0.0));

      // sort by angle
      Collections.sort(ArmConstraints);  

    }
  }

  public static class Grasper {
   public static final int GrasperMotorId = 3;
   
  }

  public static class Climber {
    public static final double TestExtendOutput = 1;
    public static final double TestRetractOutput = 1;
  }

  public static class Misc {
    public static final double pi = 3.14159;  
    public static final double inchesToMeters = 0.0254; //multiple inches to get meters
    public static final double degreeToRadian = 0.0174;
  }

  public static class CAN {
    // Timeout constants
    public static final int kLongCANTimeoutMs = 100;
    public static final int kCANTimeoutMs = 10;

  }

  public static class SwerveConstants { // constants related to swerve system
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    public static final boolean invertYAxis = true;
    public static final boolean invertXAxis = false;
    public static final boolean invertRotateAxis = false;

    /* Swerve Module Locations on the Robot */
    /*
         public static final Translation2d[] swerveModuleLocations = {
      new Translation2d(
        RobotDimensions.wheelBase/2.0, RobotDimensions.trackWidth/2.0
      ),
      new Translation2d(
        RobotDimensions.wheelBase/2.0, -RobotDimensions.trackWidth/2.0
      ),
      new Translation2d(
        -RobotDimensions.wheelBase/2.0, RobotDimensions.trackWidth/2.0
      ),
      new Translation2d(
        -RobotDimensions.wheelBase/2.0, -RobotDimensions.trackWidth/2.0
      )
    };
     
     */
    public static final Translation2d[] swerveModuleLocations = {
      new Translation2d(
        RobotDimensions.wheelBase/2.0, -RobotDimensions.trackWidth/2.0
      ),
      new Translation2d(
        -RobotDimensions.wheelBase/2.0, -RobotDimensions.trackWidth/2.0
      ),
      new Translation2d(
        RobotDimensions.wheelBase/2.0, RobotDimensions.trackWidth/2.0
      ),

      new Translation2d(
        -RobotDimensions.wheelBase/2.0, RobotDimensions.trackWidth/2.0
      ),

    };

    /* Swerve Drive Kinematics based on the module locations */
    public static final SwerveDriveKinematics swerveKinematics = 
            new SwerveDriveKinematics(swerveModuleLocations);

    /* Swerve Drive Motor PID Values */
    public static final double driveKP = 0.05;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Swerve Drive Motor Characterization Values */
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    // Dpad Only Driving Control for precise tuning
    public static final double simpleSwerveDriveSpeed = .7;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.3;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKF = 0.0;
    
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    // Gear Ratios - https://www.swervedrivespecialties.com/products/mk4i-swerve-module#:~:text=The%20steering%20gear%20ratio%20of,is%20150%2F7%3A1.
    // Stearing Gear Ratio is 150/7:1 ~ 21.43
    // Drive Gear Ratio Varies by Speed 6.12 for L3
    public static final double driveGearRatio = 6.12;
    public static final double angleGearRatio = 21.43;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Swerve Motor Inverts */
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = true;

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 5.3; // meters per second
    public static final double maxAngularVelocity = 15.0; //?

    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    // SwerveAdjustment Value. This Value is in meters
    public static final double swerveAdjustValue = .5;

    /* Swerve Motor */
    public static final class Motor 
    {
      public static final int kFalconEncoderCPR = 2048;
      public static final int kCANCoderCPR = 4096;
      public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
      public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

      public static final double kDriveDistancePerPulse =
      (wheelDiameter * Math.PI) / (kFalconEncoderCPR * driveGearRatio);
      public static final double kTurnDistancePerPulse =
          360.0 / (kFalconEncoderCPR * angleGearRatio);
      public static final double kTurningEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

      public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
      public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
      public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

      public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
      public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
      }

    /* Swerve Specific Auto Constants */
    public static final class AutoConstants 
    {

      public static final class AutoPilot {
        TuneableNumber mXTolerance = new TuneableNumber("XTolerance", .2);
        TuneableNumber mYTolerance = new TuneableNumber("YTolerance", .2);
        TuneableNumber mOmegaTolerance = new TuneableNumber("OmegaTolerance", .2);


        /* Teleop  */
        public static final double TeleopDriveVelocity = 2.5;
        public static final double TeleopDriveAcceleration = 8;
        public static final double TeleopThetaVelocity = 8;
        public static final double TeleopThetaAcceleration = .8;
        public static final double TeleopMaxSpeed = 2;


        /* Charging Station */
        public static final double CSDriveVelocity = 1.1;
        public static final double CSDriveAcceleration = 3;
        public static final double CSThetaVelocity = 2;
        public static final double CSThetaAcceleration = .5;
        public static final double CSMaxSpeed = .5;      
      }


      public static final double kSlowSpeedMetersPerSecond = 1.7;
      public static final double kSlowAccelerationMetersPerSecondSquared = 2.0;

      public static final double kMaxSpeedMetersPerSecond = 2.2; 
      public static final double kMaxAccelerationMetersPerSecondSquared = 2.3;
      
      public static final double kSlowMaxAngularSpeedRadiansPerSecond = 0.8 * Math.PI;
      public static final double kSlowMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kSlowMaxAngularSpeedRadiansPerSecond, 2);

      public static final double kMaxAngularSpeedRadiansPerSecond = 1.2 * Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 5;

       // Constraint for the motion profilied robot angle controller
       public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
         new TrapezoidProfile.Constraints(
               kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

       // Constraint for the motion profilied robot angle controller
       public static final TrapezoidProfile.Constraints kSlowThetaControllerConstraints =
         new TrapezoidProfile.Constraints(
               kSlowMaxAngularSpeedRadiansPerSecond, kSlowMaxAngularSpeedRadiansPerSecondSquared);
       
               public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed, double endSpeed) {
                TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
                config.setKinematics(Constants.SwerveConstants.swerveKinematics);
                config.setStartVelocity(startSpeed);
                config.setEndVelocity(endSpeed);
                config.addConstraint(new CentripetalAccelerationConstraint(3.0));
                return config;
            }
    
            // Trajectory Speed Configs
            public static final TrajectoryConfig defaultSwerveTrajectorySpeedConfig =
                    new TrajectoryConfig(
                            kMaxSpeedMetersPerSecond,
                            kMaxAccelerationMetersPerSecondSquared)
                            .setKinematics(Constants.SwerveConstants.swerveKinematics);
    
            public static final TrajectoryConfig slowSwerveSpeedConfig =
                    new TrajectoryConfig(
                    kSlowSpeedMetersPerSecond,
                    kSlowAccelerationMetersPerSecondSquared)    
                    .setKinematics(Constants.SwerveConstants.swerveKinematics)
                            .setStartVelocity(0)
                            .setEndVelocity(0); 
    }; 
  }
}