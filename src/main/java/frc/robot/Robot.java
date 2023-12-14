// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.Shooter.ShooterState;
import frc.robot.TuneableNumber;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


  }

OperatorInterface mOperatorInterface = OperatorInterface.getInstance();



  //Setting the speed
TuneableNumber TopEjectSpeed = new TuneableNumber("Top Eject Speed", -0.1);
TuneableNumber BottomEjectSpeed = new TuneableNumber("Bottom Eject Speed", -0.1);
//3 doesn't exist because motors 2 and 3 are paired
TuneableNumber HighIntakeSpeed = new TuneableNumber("High Intake Speed", -0.1);
TuneableNumber MidIntakeSpeed = new TuneableNumber("Mid Intake Speed", -0.1);
TuneableNumber LowIntakeSpeed = new TuneableNumber("Low Intake Speed", 0.1);


//Setting the motors
com.ctre.phoenix6.hardware.TalonFX TopEject = new TalonFX(6);
TalonFX BottomEject1 = new TalonFX(5);
TalonFX BottomEject2 = new TalonFX(4);
TalonFX HighIntake = new TalonFX(3);
TalonFX MidIntake = new TalonFX(2);
TalonFX LowIntake = new TalonFX(1);


  @Override
  public void testPeriodic() {

    //Hold the right trigger
    if(mOperatorInterface.getBallerBalling()){
    //Trying to pair the two bottom motors to spin the same way and share a single value on the smart dashboard
    double BottomEjectSpeed1 = BottomEjectSpeed.get();
     double BottomEjectSpeed2 = BottomEjectSpeed.get() * -1;

     TopEject.setControl(new DutyCycleOut( TopEjectSpeed.get()));
     BottomEject1.setControl(new DutyCycleOut( BottomEjectSpeed1));
     BottomEject2.setControl(new DutyCycleOut( BottomEjectSpeed2));
     HighIntake.setControl(new DutyCycleOut(HighIntakeSpeed.get()));
     MidIntake.setControl(new DutyCycleOut( MidIntakeSpeed.get()));
     LowIntake.setControl(new DutyCycleOut( LowIntakeSpeed.get()));
    }else {
      TopEject.setControl(new DutyCycleOut( 0));
      BottomEject1.setControl(new DutyCycleOut( 0));
      BottomEject2.setControl(new DutyCycleOut( 0));
      HighIntake.setControl(new DutyCycleOut(0));
      MidIntake.setControl(new DutyCycleOut( 0));
      LowIntake.setControl(new DutyCycleOut( 0));
    }

    if (mOperatorInterface.getShooterRunning()){
      setShooterState(ShooterState.Running);
    }

    if (mOperatorInterface.stopShooter()){
      setShooterState(ShooterState.Stopped);
    }

  }
}