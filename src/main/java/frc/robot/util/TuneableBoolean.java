package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TuneableBoolean {
    private static final String tableKey = "TuneableBooleans";

    private String key;
    private Boolean defaultValue;
    private Boolean lastHasChangedValue = defaultValue; 

     /**
     * Create a new TuneableNumber
     * 
     * @param dashboardKey Key on dashboard
     */
    public TuneableBoolean(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
      }
  
       /**
     * Create a new TuneableNumber with the default value
     * 
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TuneableBoolean(String dashboardKey, Boolean defaultValue) {
      this(dashboardKey);
      setDefault(defaultValue);
    }
  
    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public Boolean getDefault() {
      return defaultValue;
    }
  
    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    public void setDefault(Boolean defaultValue) {
      this.defaultValue = defaultValue;
      if (Constants.tuningMode) {
        // This makes sure the data is on NetworkTables but will not change it
        SmartDashboard.putBoolean(key,
            SmartDashboard.getBoolean(key, defaultValue));
      } else {
        //@TODO - Delete Key
        //SmartDashboard.delete(key);
      }
    }
  
    /**
     * Get the current value, from dashboard if available and in tuning mode
     * 
     * @return The current value
     */
    public boolean get() {
      return Constants.tuningMode ? SmartDashboard.getBoolean(key, defaultValue)
          : defaultValue;
    }
  
    /**
     * Checks whether the number has changed since our last check
     * 
     * @return True if the number has changed since the last time this method was called, false
     *         otherwise
     */
    public boolean hasChanged() {
        boolean currentValue = get();
      if (currentValue != lastHasChangedValue) {
        lastHasChangedValue = currentValue;
        return true;
      }
  
      return false;
    }
}
