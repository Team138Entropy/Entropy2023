package frc.robot;

public class SpeedLookupConfig {
    
    double TopEjectSpeed;
    double BottomEjectSpeed;
    double HighIntakeSpeed;
    double MidIntakeSpeed;
    double LowIntakeSpeed;
    double Distance;

    public SpeedLookupConfig(){}
    public SpeedLookupConfig(double dis, double TopEject, double BottomEject, double HighIntake, double MidIntake, double LowIntake ) {
        Distance = dis;
        TopEject = TopEjectSpeed;
        BottomEject = BottomEjectSpeed; 
        HighIntake = HighIntakeSpeed;
        MidIntake = MidIntakeSpeed;
        LowIntake = LowIntakeSpeed;

    }



}