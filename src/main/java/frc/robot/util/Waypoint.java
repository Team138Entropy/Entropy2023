package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Waypoint {

    // Poses Based on Alliance Color
    private Pose2d[] mPoses;

    public Waypoint()
    {
        // Pose for each alliance
        mPoses = new Pose2d[2];

        // Default each pose
        for(var i = 0; i < 2; ++i)
        {
            mPoses[i] = new Pose2d();
        }
    }

    // Set Pose 
    public void setPose(Pose2d pose, Alliance color)
    {
        if(color != Alliance.Invalid)
        {
            mPoses[getIndex(color)] = pose;
        }
    }

    // Get Pose
    public Pose2d getPose(Alliance color)
    {
        Pose2d result = null;
        if(color != Alliance.Invalid)
        {
            result = mPoses[getIndex(color)];
        }
        return result;
    }

        // Set Translation 
        public void setTranslation(Translation2d trans, Alliance color)
        {
            if(color != Alliance.Invalid)
            {
                mPoses[getIndex(color)] = new Pose2d(
                    trans,
                    mPoses[getIndex(color)].getRotation()
                );
            }
        }

    // Set Rotation 
    public void setRotation(Rotation2d rot, Alliance color)
    {
        if(color != Alliance.Invalid)
        {
            mPoses[getIndex(color)] = new Pose2d(
                mPoses[getIndex(color)].getTranslation(),
                rot
            );
        }
    }

    // Get Color for Alliance Enum
    private int getIndex(Alliance color)
    {
        return (Alliance.Blue == color ? 0 : 1);
    }

}
