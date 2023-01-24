package frc.robot;

public class Enums {
    
    // Type of Jogging
    public enum JogDirection {
        FORWARD, 
        REVERSE, 
        STOP
    };

    // scoring nodes
    public enum TargetedPositions {
        //bottom, middle, and top are refering to the grids not the scoring hights
        //top and bottom are fliped on one side 
        NONE,
        GRID_BOTTOM_1,
        GRID_BOTTOM_2,
        GRID_BOTTOM_3,
        GRID_MIDDLE_1,
        GRID_MIDDLE_2,
        GRID_MIDDLE_3,
        GRID_TOP_1,
        GRID_TOP_2,
        GRID_TOP_3,
        RED_SUBSTATION_LEFT,
        RED_SUBSTATION_RIGHT,
        BLUE_SUBSTATION_LEFT,
        BLUE_SUBSTATION_RIGHT;
    }

    // arm targets
    public enum ArmTargets {
        //NAME_OF_POSITION(armAngle, armExtension),
        //THESE ARE PLACEHOLDER VALUES
        TOP_SCORING_FRONT(15,100),
        MID_SCORING_FRONT(345,60),
        LOW_SCORING_FRONT(320,40),
        INTAKE_FRONT(10,0),
        TOP_SCORING_BACK(170,100),
        MID_SCORING_BACK(195,60),
        LOW_SCORING_BACK(220,40),
        INTAKE_BACK(170,0),
        INTAKE_GROUND_FRONT(320,0),
        INTAKE_GROUND_BACK(230,0),
        SAFE(90,0),
        START(315, 0),
        HOME_BACKSIDE(235, 0),
        HOME_FRONTSIDE(315, 0),
        NONE(315,0);



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
}
