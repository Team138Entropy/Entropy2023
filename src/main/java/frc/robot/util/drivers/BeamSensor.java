package frc.robot.util.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

// Beam Sensor
// Digital Output based Beam Sensor
// Supports Sampling to cut down on false samples
public class BeamSensor {

    // DIO Beam Sensor Reference
    private final DigitalInput mInput;

    // Amount of Positive Samples needed to mark the beam broken
    private final int mTargetPositiveSampleCount;

    private int mCurrentPositiveSamples;

    public BeamSensor(int ioPort)
    {
        this(ioPort, 1);
    }

    public BeamSensor(int ioPort, int positiveSampleCount)
    {
        mInput = new DigitalInput(ioPort);
        mTargetPositiveSampleCount = positiveSampleCount;
        mCurrentPositiveSamples = 0;
    }

    // Continuous Update Loop
    // Evalutes the current DIO State
    public void update()
    {
        // get() of DIO returns true if sees other beam, and false if not
        if(!mInput.get())
        {
            // Beam is Broken! 
            mCurrentPositiveSamples++;
        }else 
        {
            // Beam is not broken
            // Reset samples
            mCurrentPositiveSamples = 0;
        }
    }

    // Returns True if the Beam is Broken
    public boolean isBeamBroken()
    {
        return (mCurrentPositiveSamples >= mTargetPositiveSampleCount);
    }
}
