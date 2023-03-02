package frc.robot;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Enums {
    
    // Type of Jogging
    public enum JogDirection {
        FORWARD, 
        REVERSE, 
        STOP
    };

    // scoring nodes
    public enum TargetedPositions {
        //Numbered left to right as if you are standing behind it
        NONE,
        GRID_1,
        GRID_2,
        GRID_3,
        GRID_4,
        GRID_5,
        GRID_6,
        GRID_7,
        GRID_8,
        GRID_9,
        SUBSTATION_LEFT,
        SUBSTATION_RIGHT;
    }

    // arm targets
    public enum ArmTargets {
        //NAME_OF_POSITION(armAngle, armExtension),
        //THESE ARE PLACEHOLDER VALUES
        TOP_SCORING_FRONT(10, Constants.Arm.MaxExtensionPosition),
        TOP_SCORING_FRONT_CUBE(1, Constants.Arm.MaxExtensionPosition),
        MID_SCORING_FRONT(-3,Constants.Arm.MinExtensionPosition),
        MID_SCORING_FRONT_CUBE(-15,Constants.Arm.MinExtensionPosition),

        LOW_SCORING_FRONT(-50,Constants.Arm.MinExtensionPosition),
        INTAKE_FRONT(-3,Constants.Arm.MinExtensionPosition),
        POST_INTAKE_FRONT(-1,Constants.Arm.MinExtensionPosition),
        TOP_SCORING_BACK(170,0),
        MID_SCORING_BACK(173,Constants.Arm.MaxExtensionPosition),
        LOW_SCORING_BACK(207,186000),
        INTAKE_BACK(169,0),
        POST_INTAKE_BACK(167,0),
        INTAKE_GROUND_FRONT(-50,143000),
        INTAKE_GROUND_BACK(217,208000),
        SAFE(90,0),
        START(233, 0),
        HOME_BACKSIDE(220, 0),
        HOME_FRONTSIDE(-50, 0),

        NONE(90,0);


        public final double armAngle;
        public final double armExtend;

        private ArmTargets(double angle, double extend) {
            this.armAngle = angle;
            this.armExtend = extend;
        }

        private double getArmAngle(){
            return armAngle;
        }

        private double getArmExtend(){
            return armExtend;
        }

    }

    public enum TargetedObject {
        CONE,
        CUBE;
    }

    public enum cameraType {
        FRONT_CAMERA,
        BACK_CAMERA,
        GRASPER_CAMERA;
      }

    // Swerve Cardinal Directions
    // These are SNAP Directions that will allow the swerve system
    // to auto turn to them
    public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180),
        TEST(45),

        FAR_FENDER(143),
        RIGHT_FENDER(233),
        LEFT_FENDER(53),
        CLOSE_FENDER(323);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    // Swerve Rotation
    // Useful for getting rotations
    public enum SwerveRotation {
        FRONT_FACING_FORWARD(0),
        FRONT_FACING_FORWARD_LEFT_ANGLE(45),
        
        FRONT_FACING_GRID(180),
        BACK_FACING_GRID(0),
        FRONT_FACING_RIGHT(90),
        FRONT_FACING_LEFT(90)
        ;

        public final double degrees; 

        // Get Degrees Value to Rotation
        public Rotation2d getRotation()
        {
            return Rotation2d.fromDegrees(degrees);
        }

        SwerveRotation(double degrees) {
            this.degrees = degrees;
        }
    }

    // SwerveQuickAdjust
    //  Similar to the Swerve Snapping System, 
    //      Swerve system will button adjust
    public enum SwerveQuickAdjust {
        FORWARD(new Pose2d(0, 0, new Rotation2d())),
        BACKWARD(new Pose2d(0, 0, new Rotation2d())),
        LEFT(new Pose2d(0, 0, new Rotation2d())),
        RIGHT(new Pose2d(0, 0, new Rotation2d())),
        NONE(new Pose2d(0, 0, new Rotation2d()));

        public final Pose2d targetPose;

        SwerveQuickAdjust(Pose2d pose)
        {
            this.targetPose = pose;
        }
    }

    // Camera IDs
    public enum Cameras {
        FRONT_SIDE(0),
        BACK_SIZE(1),
        ARM(2);

        public final int camID;

        Cameras(int cid) {
            this.camID = cid;
        }
    }

    // Names to April Tag IDs
    public enum AprilTagName {
        
    }

    // Game Object
    public enum GamePiece {
        Cone,
        Cube
    };

    // Game Walls
    public enum GameWalls {
        AllianceWall,
        OpponentWall
    };

    // Arm Control Type
    public enum ArmControlType {
        Simple, 
        Advanced
    };

    public enum ArmRotationSpeed {
        DEFAULT(15, 20),
        OVER_TOP_FORWARDS(15, 15),
        OVER_TOP_BACKWARDS(15, 10);

        public final double velocity;
        public final double acceleration;
        
        ArmRotationSpeed(double vel, double accel)
        {
            this.velocity = vel;
            this.acceleration = accel;
        }
    }
}
