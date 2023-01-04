package frc.robot.util;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Entropy !38s Logging Functions
 * These functions use WPILibs's DataLogManager
 */
public final class Logger {
    
    // Log String Value to the DataLogManager
    public static void log(String value)
    {
        DataLogManager.log(value);
    }


    public static void log(String className, String value)
    {
        DataLogManager.log(className + ":" + value);
    }
    

    // could automatically get calling class:
    //      https://stackoverflow.com/questions/11306811/how-to-get-the-caller-class-in-java

}
