package frc.robot.util.physics;



public class ArmConstraint implements Comparable<ArmConstraint> {
  
    // Arm Constraint Type
    public enum ArmConstraintType {
        None,              // No Constraint
        ExtensionMaximum,  // Extension cannot be extended this extension value!
        ExtensionMinimum   // Extension cannot be pulled in beyond this value
    };

    public final double Angle;
    public final ArmConstraintType Type; 
    public final double Value;

    public ArmConstraint(double ang, ArmConstraintType t, double val)
    {
        Angle = ang;
        Type = t; 
        Value = val;
    }

    // Compare Function
    /*
     * a negative int if this < that
0 if this == that
a positive int if this > that
     */
    public int compareTo(ArmConstraint otherConstraint)
    {
        int result = 0;
        if(Angle < otherConstraint.Angle)
        {
            result = -1;
        }else if(Angle > otherConstraint.Angle)
        {
            result = 1;
        }
        return result;
    }


}
