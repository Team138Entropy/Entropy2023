package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LedManager {
  private static LedManager mInstance;

  public static LedManager getInstance(){
    if (mInstance == null) {
      mInstance = new LedManager();
    }
    return mInstance;
  }

  //idk what the intent of this requirment is so im going to use it as a "robot state" type thing
  public enum entropyColor{
    ROBOT_DISABLED(Color.kBlack, Color.kGold),
    AUTONOMOUS(Color.kBlack, Color.kGold),
    PIECE_ACQUIRED(Color.kGreen),
    LOW_BATTERY(Color.kOrange),
    CODE_STARTING(Color.kWhite),
    ROBOT_TEST(Color.kLightYellow, Color.kBlack),
    ROBOT_FLIPPED(Color.kRed, Color.kBlack);
      
      
    public final Color color1;
    public final Color color2;

    private entropyColor(Color firstColor, Color secondColor){
      this.color1 = firstColor;
      this.color2 = secondColor;
    }

    private entropyColor(Color color){
      this.color1 = color;
      this.color2 = null;
    }
  }

  //LED object
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  //values for setting LEDs
  private static final int length = 43;
  private static final double breathDuration = 1.0;

  //main color var
  private entropyColor currentColor;

  private LedManager(){
    //port:0 placeholder
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

  public void setColor(entropyColor color){
    currentColor = color;
  }

  public entropyColor getColor(){
    return currentColor;
  }

  //LED control styles
  private void solid(Color color) {
    for (int i = 0; i < length; i++) {
      buffer.setLED(i, color);
    }
  }
    
  private void strobe( Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(on ? color : Color.kBlack);
  }
    
  private void stripes(List<Color> colors, int mlength, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * mlength * colors.size());
    for (int i = 0; i < length; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / mlength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  //update function 
  public synchronized void periodic(){
    if(currentColor == entropyColor.ROBOT_DISABLED){
      stripes(List.of(currentColor.color1,currentColor.color2), 3, 5);
    }else if(currentColor == entropyColor.AUTONOMOUS){
      stripes(List.of(currentColor.color1,currentColor.color2), 3, 5);
    }else if(currentColor == entropyColor.PIECE_ACQUIRED){
      solid(currentColor.color1);
    }else if(currentColor == entropyColor.LOW_BATTERY){
      solid(currentColor.color1);
    }else if(currentColor == entropyColor.CODE_STARTING){
      strobe(currentColor.color1, .2);
    }else if(currentColor == entropyColor.ROBOT_TEST){
      stripes(List.of(currentColor.color1,currentColor.color2), 5, 5);
    }else if(currentColor == entropyColor.ROBOT_FLIPPED){
      breath(currentColor.color1, currentColor.color2, 1);
    }
  }
}
