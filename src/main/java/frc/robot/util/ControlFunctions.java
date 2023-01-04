package frc.robot.util;

public class ControlFunctions {
    

    public static double getSpeed(
        double errorAngle,
        double maxSpeed, 
        double minSpeed,
        double maxAngle,
        double minAngle
    ){
        double slope = (maxSpeed - minSpeed)/(maxAngle - minAngle);
        double speed = errorAngle * slope;

        if(speed > 0){
            if(errorAngle < minAngle){
                speed = 0; //within range
            }else if(errorAngle > maxAngle){
                speed = maxSpeed; // maximum speed, get in fast
            }else if(speed > maxSpeed){
                speed = maxSpeed;
            }
        }else if(speed < 0){
            if(errorAngle > -minAngle){
                speed = 0;
            }else if(errorAngle < -maxAngle){
                speed = maxSpeed;
            }else if(speed < -maxSpeed){
                speed = -maxSpeed;
            }
        } 
        speed = speed * -1;
        return speed;
    }
}
