package frc.robot.util;

import java.util.List;

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

  //a entropyColor is essentially the state of the robot and the colors displayed when in that state
  public enum entropyColor{
    //note: even if two or more entropyColors have the same colors attached, they are seperate because they are displayed in different ways
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
  private final AddressableLED mLeds;
  private final AddressableLEDBuffer mBuffer;

  //values for setting LEDs
  private static final int mLength = 43;
  private static final double mBreathDuration = 1.0;

  //main color var
  private entropyColor mCurrentColor;

  private LedManager(){
    //port:0 placeholder
    mLeds = new AddressableLED(0);
    mBuffer = new AddressableLEDBuffer(mLength);
    mLeds.setLength(mLength);
    mLeds.setData(mBuffer);
    mLeds.start();
  }

  public void setColor(entropyColor color){
    mCurrentColor = color;
  }

  public entropyColor getColor(){
    return mCurrentColor;
  }

  //LED control styles:
  
  private void solid(Color color) {
    for (int i = 0; i < mLength; i++) {
      mBuffer.setLED(i, color);
    }
  }
    
  private void strobe( Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(on ? color : Color.kBlack);
  }
    
  private void stripes(List<Color> colors, int mlength, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * mlength * colors.size());
    for (int i = 0; i < mlength; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / mlength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      mBuffer.setLED(i, colors.get(colorIndex));
    }
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % mBreathDuration) / mBreathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }
  
  //update function 
  public synchronized void periodic(){
    switch(mCurrentColor){
      case ROBOT_DISABLED:
        stripes(List.of(mCurrentColor.color1,mCurrentColor.color2), 3, 5);
       break;
      case AUTONOMOUS:
        stripes(List.of(mCurrentColor.color1,mCurrentColor.color2), 3, 5);
        break;
      case PIECE_ACQUIRED:
        solid(mCurrentColor.color1);
        break;
      case LOW_BATTERY:
        solid(mCurrentColor.color1);
        break;
      case CODE_STARTING:
        strobe(mCurrentColor.color1, .2);
        break;
      case ROBOT_TEST:
        stripes(List.of(mCurrentColor.color1,mCurrentColor.color2), 5, 5);
        break;
      case ROBOT_FLIPPED:
        breath(mCurrentColor.color1, mCurrentColor.color2, 1);
        break;
    }
    mLeds.setData(mBuffer);
  }
}
