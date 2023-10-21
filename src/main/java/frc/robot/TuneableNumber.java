package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TuneableNumber {
    private static final String tableKey = "TuneableNumbers";

    private String key;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;
  
    /**
     * Create a new TuneableNumber
     * 
     * @param dashboardKey Key on dashboard
     */
    public TuneableNumber(String dashboardKey) {
      this.key = tableKey + "/" + dashboardKey;
    }

     /**
   * Create a new TuneableNumber with the default value
   * 
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TuneableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   * 
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   * 
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (true) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putNumber(key,
          SmartDashboard.getNumber(key, defaultValue));
    } else {
      // @TODO - Figure out how to delete key
      // SmartDashboard.delete(key);
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   * 
   * @return The current value
   */
  public double get() {
    return true ? SmartDashboard.getNumber(key, defaultValue)
        : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   * 
   * @return True if the number has changed since the last time this method was called, false
   *         otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }
}