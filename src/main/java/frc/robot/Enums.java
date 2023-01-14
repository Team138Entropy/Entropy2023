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
