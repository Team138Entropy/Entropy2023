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
        TOP_SCORING_FRONT(80,100),
        MID_SCORING_FRONT(65,60),
        LOW_SCORING_FRONT(50,40),
        INTAKE_FRONT(80,0),
        TOP_SCORING_BACK(80,100),
        MID_SCORING_BACK(65,60),
        LOW_SCORING_BACK(50,40),
        INTAKE_BACK(80,0),
        SAFE(90,0),
        NONE(0,0);



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

    public enum targetColor {
        //color values are temperary
        CONE_COLOR(10),
        CUBE_COLOR(20);

        public double colorValue;

        private targetColor(double colorValue) {
            this.colorValue = colorValue;
        }
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
}
