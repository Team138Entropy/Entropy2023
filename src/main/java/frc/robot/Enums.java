package frc.robot;

public class Enums {
    
    // Type of Jogging
    public enum JogDirection {
        FORWARD, 
        REVERSE, 
        STOP
    };

    // scoring nodes
    public enum scoringNodes {
        //bottom, middle, and top are refering to the grids not the scoring hights
        NONE,
        BOTTOM_1,
        BOTTOM_2,
        BOTTOM_3,
        MIDDLE_1,
        MIDDLE_2,
        MIDDLE_3,
        TOP_1,
        TOP_2,
        TOP_3;
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
        SAFE(90,0);

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
