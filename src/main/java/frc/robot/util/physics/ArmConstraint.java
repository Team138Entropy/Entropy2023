package frc.robot.util.physics;

import java.util.Vector;

import frc.robot.Enums;

public class ArmConstraint implements Comparable<ArmConstraint> {
  
    // Arm Constraint Type
    public enum ArmConstraintType {
        None,              // No Constraint
        ExtensionMaximum,  // Extension cannot be extended beyond this extension value!  ONLY ONE BEING USED RIGHT NOW
        ExtensionMinimum   // Extension cannot be pulled in beyond this value
    };

    public final double Angle;
    public final ArmConstraintType Type; 
    public final double Value;

    // This Constraint Only Applies if targeting a Position
    public final boolean ConditionalConstraint; 

    // Conditional Targets
    private final Vector<Enums.ArmTargets> ConditionalTargets = new Vector<Enums.ArmTargets>();

    // Basic Constructor
    public ArmConstraint(double ang, ArmConstraintType t, double val)
    {
        Angle = ang;
        Type = t; 
        Value = val;
        ConditionalConstraint = false;
    }

    // Conditional Arm Constrant
    // Conditional 
    public ArmConstraint(double ang, ArmConstraintType t, double val, Enums.ArmTargets[] condArmTargets)
    {
        Angle = ang;
        Type = t; 
        Value = val;
        ConditionalConstraint = true;

        // Add Conditional Targets 
        for(int i = 0; i < condArmTargets.length; ++i)
        {
            ConditionalTargets.add(condArmTargets[i]);
        }
    }

    // If this a valid constraint for the current target
    // If not a conditional restraint, always returns true
    public boolean isConstraintForTarget(Enums.ArmTargets target)
    {
        boolean targetConstraint = false;
        if(ConditionalConstraint)
        {
            // This constraint is conditional
            // if it exsits in the conditional targets, then it applies
            targetConstraint = ConditionalTargets.contains(target);
        }else {
            // not conditional constraint 
            // this constraint should apply to any target
            targetConstraint = true;
        }
        return targetConstraint;
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
