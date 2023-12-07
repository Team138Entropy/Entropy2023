package frc.robot;

public class VelocityLookupTable {
    


/* 
Distance in meters, speed in RPS
First number is distance, then motors in order: TopEject, BottomEject, HighIntake, MidIntake, LowIntake
*/
    SpeedLookupConfig[] mVelocityLookupTable = {
       new SpeedLookupConfig(5, 0, 0, 0, 0, 0),
       new SpeedLookupConfig(10, 0, 0, 0, 0, 0),
       new SpeedLookupConfig(15, 0, 0, 0, 0, 0)
    };

    VelocityLookupTable() {}


  public double linearInterpolate(double x, double x0, double y0, double x1, double y1) {
    return (y0 * (x1 - x) + y1 * (x - x0)) / (x1 - x0);
  }

  public SpeedLookupConfig getSpeedFromDistance(double distance) {


      for (int i = 0; i < mVelocityLookupTable.length; i++) {
        double thisDistance = mVelocityLookupTable[i].Distance;
        double thisSpeed = 1;

        if (thisDistance == distance) {
          return mVelocityLookupTable[i];
        }


        // make a new speed config
          SpeedLookupConfig slowestSpeed = mVelocityLookupTable[0];
          SpeedLookupConfig fastestSpeed = mVelocityLookupTable[mVelocityLookupTable.length - 1];
          SpeedLookupConfig linearSolved = new SpeedLookupConfig();
          linearSolved.Distance = distance; 
          linearSolved.TopEjectSpeed = linearInterpolate(fastestSpeed.TopEjectSpeed, slowestSpeed.TopEjectSpeed, i, thisDistance, thisSpeed);
          linearSolved.BottomEjectSpeed = linearInterpolate(fastestSpeed.BottomEjectSpeed, slowestSpeed.BottomEjectSpeed, i, thisDistance, thisSpeed);
          linearSolved.HighIntakeSpeed = linearInterpolate(fastestSpeed.HighIntakeSpeed, slowestSpeed.HighIntakeSpeed, i, thisDistance, thisSpeed);
          linearSolved.MidIntakeSpeed = linearInterpolate(fastestSpeed.MidIntakeSpeed, slowestSpeed.MidIntakeSpeed, i, thisDistance, thisSpeed);
          linearSolved.LowIntakeSpeed = linearInterpolate(fastestSpeed.LowIntakeSpeed, slowestSpeed.LowIntakeSpeed, i, thisDistance, thisSpeed);


            return linearSolved;
        }
        return null;

  }
}










