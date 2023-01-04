package frc.robot.util.simulation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
//import edu.wpi.first.wpilibj.simulation.
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.DriveSignal;

//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
/**
 * DriveSimConfig
 * All Sim objects are allowed to remain in codebase 
 * when deployed to the RoboRIo
 */
public class DiffDriveSimSystem {

    // Differential Drive Sim System
    DifferentialDrivetrainSim mDriveSim;

    
    // Encoder Sim
    private final EncoderSim mLeftEncoderSim;
    private final EncoderSim mRightEncoderSim;

    // Gyro Sim
    private final AnalogGyroSim mGyroSim;

    /* Drive Sim Constructor */
    public DiffDriveSimSystem()
    {
        mLeftEncoderSim = null;
        mRightEncoderSim = null;
        mGyroSim = null;

        //setup the drive sim
        initDrivetrainSim();
    }

    /* Drive Sim Constructor */
    public DiffDriveSimSystem(AnalogGyro gyro, 
                Encoder leftEncoder, Encoder rightEncoder)
    {
        // Set References to real encoders
        mLeftEncoderSim = new EncoderSim(leftEncoder);
        mRightEncoderSim = new EncoderSim(rightEncoder);
        mGyroSim = new AnalogGyroSim(gyro);


        // setup the drive sim
        initDrivetrainSim();
    }

    private void initDrivetrainSim()
    {
        // Create Drive Sim
        mDriveSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),       // 2 falcon500 motors on each side of the drivetrain.
            7.29,                    // 7.29:1 gearing reduction.
            7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
            60.0,                    // The mass of the robot is 60 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            0.7112,                  // The track width is 0.7112 meters.
            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );
    }

    /* Simulate Current Input */
    public synchronized void updateDrive(DriveSignal ds)
    {
        // Set the inputs to the system. Input is expected to be on the scale of [-1, 1]
        mDriveSim.setInputs(ds.getLeft(), ds.getRight());

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        mDriveSim.update(0.02);

        // Update all of our sensors. (if not null)
        if(null != mLeftEncoderSim)
        {
            mLeftEncoderSim.setDistance(mDriveSim.getLeftPositionMeters());
            mLeftEncoderSim.setRate(mDriveSim.getLeftVelocityMetersPerSecond());
        }
        if(null != mRightEncoderSim)
        {
            mRightEncoderSim.setDistance(mDriveSim.getRightPositionMeters());
            mRightEncoderSim.setRate(mDriveSim.getRightVelocityMetersPerSecond());
        }
        if(null != mGyroSim)
        {
            mGyroSim.setAngle(-mDriveSim.getHeading().getDegrees());
        }

        // Update Smart Dashboard
        String simKey = "Sim/";
        SmartDashboard.putNumber(simKey + "Drive Left Signal", ds.getLeft());
        SmartDashboard.putNumber(simKey + "Drive Right Signal", ds.getRight());
        SmartDashboard.putNumber(simKey + "Sim Drive Left Meters Per Second", mDriveSim.getLeftVelocityMetersPerSecond());
        SmartDashboard.putNumber(simKey + "Sim Drive Left Feet Per Second", mDriveSim.getLeftVelocityMetersPerSecond());
        SmartDashboard.putNumber(simKey + "Sim Drive Right Meters Per Second", mDriveSim.getRightVelocityMetersPerSecond());
        SmartDashboard.putNumber(simKey + "Sim Drive Right Feet Per Second", mDriveSim.getRightVelocityMetersPerSecond());    
    }




}
