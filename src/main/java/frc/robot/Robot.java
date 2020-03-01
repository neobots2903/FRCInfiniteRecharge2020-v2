/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climb2903;
import frc.robot.subsystems.NavX2903;
import frc.robot.subsystems.Shooter2903;
import frc.robot.subsystems.SwerveDrive2903;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean autoFinished = false;

  Joystick driveJoy = new Joystick(0);
  Joystick opJoy = new Joystick(1);

  public static Climb2903 climbSubsystem;
  public static NavX2903 navXSubsystem;
  public static Shooter2903 shooterSubsystem;
  public static SwerveDrive2903 swerveDriveSubsystem;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    climbSubsystem = new Climb2903();
    navXSubsystem = new NavX2903();
    shooterSubsystem = new Shooter2903();
    swerveDriveSubsystem = new SwerveDrive2903();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  
    climbSubsystem.RetractArms();
    climbSubsystem.LowerArms();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoFinished = false;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (!autoFinished) {
      switch (m_autoSelected) {
        case kCustomAuto:
          // Put custom auto code here
          break;
        case kDefaultAuto:
        default:
          double startTime = System.currentTimeMillis();
          double driveTime = 500; //milliseconds
          while (System.currentTimeMillis() < startTime + driveTime) {
            swerveDriveSubsystem.TankDrive(-1, -1);
          }
          swerveDriveSubsystem.stopDrive();
          autoFinished = true;
          break;
      }
    }
  }

  @Override
  public void teleopInit() {
    shooterSubsystem.intakeOpen();
    shooterSubsystem.shooterBlock();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    swerveDriveSubsystem.TankDrive(driveJoy.getRawAxis(1), driveJoy.getRawAxis(5));

    shooterSubsystem.shootSpeed(opJoy.getRawAxis(3)*10);

    if (opJoy.getRawButton(1))
      shooterSubsystem.intake(1);
    else if (opJoy.getRawButton(2))
      shooterSubsystem.intake(-1);
    else
      shooterSubsystem.intake(0);

    if (opJoy.getRawButton(8)) {
      if (opJoy.getRawButton(7))
        climbSubsystem.RaiseArms();
      else if(opJoy.getRawButton(6))
        climbSubsystem.ExtendArms();
      else if (opJoy.getRawButton(5))
        climbSubsystem.RetractArms();
    }

    if(opJoy.getRawButton(3))
      shooterSubsystem.shooterUnblock();
    else if (opJoy.getRawButton(4))
      shooterSubsystem.shooterBlock();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
