package frc.robot.OI;
import edu.wpi.first.wpilibj.Joystick;


public class APacController {
    private final Joystick mController;

    public enum Buttons {
        //9(start) is a "shift" to use the other 13-28 buttons
        B(12),
        A(11),
        COIN(10),
        START(9),
        SW8(8),
        SW7(7),
        SW6(6),
        SW5(5),
        SW4(4),
        SW3(3),
        SW2(2),
        SW1(1);

        public final int id;

        Buttons(int id) {
        this.id = id;
        }
    }

    public enum Side {
        LEFT,
        RIGHT
      }
    
      public enum Axis {
        X,
        Y
      }

    // Pass in the port of the Controller
    public APacController(int portArg) {
        mController = new Joystick(portArg);
    }

    boolean getButton(Buttons button) {
        return mController.getRawButton(button.id);
    }

    double getAxis(Side side, Axis axis) {
        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        return mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
      }

}
