/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Climb2903;
import frc.robot.subsystems.LidarLite2903;
import frc.robot.subsystems.Limelight2903;
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
  public static Limelight2903 limelightSubsystem;
  public static LidarLite2903 lidarSubsystem;

  public PIDController visionTurn;

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
    limelightSubsystem = new Limelight2903();
    lidarSubsystem = new LidarLite2903(RobotMap.LidarLiteV3);
    visionTurn = new PIDController(0.1, 0, 0);

    limelightSubsystem.setLight(false);

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
    SmartDashboard.putNumber("LF Deg", swerveDriveSubsystem.LeftFront.getAbsoluteTurnDegrees());
    SmartDashboard.putNumber("LR Deg", swerveDriveSubsystem.LeftRear.getAbsoluteTurnDegrees());
    SmartDashboard.putNumber("RF Deg", swerveDriveSubsystem.RightFront.getAbsoluteTurnDegrees());
    SmartDashboard.putNumber("RR Deg", swerveDriveSubsystem.RightRear.getAbsoluteTurnDegrees());

    SmartDashboard.putBoolean("LF on zero?", swerveDriveSubsystem.LeftFront.getLimit());
    SmartDashboard.putBoolean("LR on zero?", swerveDriveSubsystem.LeftRear.getLimit());
    SmartDashboard.putBoolean("RF on zero?", swerveDriveSubsystem.RightFront.getLimit());
    SmartDashboard.putBoolean("RR on zero?", swerveDriveSubsystem.RightRear.getLimit());

    SmartDashboard.putNumber("LF FW M", swerveDriveSubsystem.LeftFront.getForwardMeters());
    SmartDashboard.putNumber("LR FW M", swerveDriveSubsystem.LeftRear.getForwardMeters());
    SmartDashboard.putNumber("RF FW M", swerveDriveSubsystem.RightFront.getForwardMeters());
    SmartDashboard.putNumber("RR FW M", swerveDriveSubsystem.RightRear.getForwardMeters());

    SmartDashboard.putNumber("Shooter Angle", shooterSubsystem.getAngle());
    SmartDashboard.putNumber("Angle Amp", shooterSubsystem.getAngleCurrent());
    SmartDashboard.putNumber("Shooter Speed", shooterSubsystem.getCurrentSpeed());
    SmartDashboard.putNumber("Shooter Speed Left", shooterSubsystem.getLeftSpeed());
    SmartDashboard.putNumber("Shooter Speed Right", shooterSubsystem.getRightSpeed());

    SmartDashboard.putBoolean("Shooter top", shooterSubsystem.getTop());
    SmartDashboard.putBoolean("Shooter bottom", shooterSubsystem.getBottom());

    SmartDashboard.putNumber("Lidar Distance", lidarSubsystem.getDistance());
    SmartDashboard.putNumber("Gyro Angle", navXSubsystem.turnAngle());
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
            swerveDriveSubsystem.TankDrive(1, 1);
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
    shooterSubsystem.zeroShooterAngle();
    swerveDriveSubsystem.zeroModulesLimit();
  }

  boolean isFieldCentric = false;
  boolean driveFieldCentricLock = false;
  boolean driveReZeroLock = false;

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double drivePower = driveJoy.getRawAxis(3) - driveJoy.getRawAxis(2); //Right Trigger - Left Trigger
    double driveAngle = swerveDriveSubsystem.joystickAngle(driveJoy.getRawAxis(0), driveJoy.getRawAxis(1)); //Left Stick
    double driveTurn = driveJoy.getRawAxis(4); //Right Stick X
    boolean driveAutoAim = driveJoy.getRawButton(2); //B
    boolean driveFieldCentric = driveJoy.getRawButton(8); //Start
    boolean driveReZero = driveJoy.getRawButton(4); //Y

    double shooterSpeed = opJoy.getRawAxis(3)*10; //Right Trigger
    double intakeSpeed = opJoy.getRawAxis(5); //Right Stick Y
    boolean shooterAuto = opJoy.getRawButton(1); //A
    boolean climbSafety = opJoy.getRawButton(8); //Start
    boolean climbRaise = opJoy.getRawButton(7); //Back
    boolean climbExtend = opJoy.getRawButton(6); //Right Bumper
    boolean climbRetract = opJoy.getRawButton(5); //Left Bumper
    boolean shooterUnblock = opJoy.getRawButton(3); //X
    boolean shooterBlock = opJoy.getRawButton(4); //Y

    if (driveAutoAim && limelightSubsystem.getTV() == 1) {
      limelightSubsystem.setLight(true);
      driveTurn = MathUtil.clamp(visionTurn.calculate(limelightSubsystem.getTX(), 0),-1,1);
    } else {
        limelightSubsystem.setLight(false);
    }

    if (driveReZero) {
      if (!driveReZeroLock) {
        swerveDriveSubsystem.zeroModulesLimit();
        driveReZeroLock = true;
      }
    } else driveReZeroLock = false;

    if (driveFieldCentric) {
      if (!driveFieldCentricLock) {
        isFieldCentric = !isFieldCentric;
        driveFieldCentricLock = true;
      }
    } else driveFieldCentricLock = false;
    
    swerveDriveSubsystem.swerveDrive(drivePower, driveAngle, driveTurn, isFieldCentric);

    if (shooterAuto) {
      shooterSubsystem.shooting(lidarSubsystem.getDistance(), 1);
    } else {
      shooterSubsystem.shootSpeed(shooterSpeed);
      shooterSubsystem.setAngle(45);
    }
    shooterSubsystem.intake(intakeSpeed);

    if (climbSafety) {
      if (climbRaise)
        climbSubsystem.RaiseArms();
      else if(climbExtend)
        climbSubsystem.ExtendArms();
      else if (climbRetract)
        climbSubsystem.RetractArms();
    }

    if(shooterUnblock)
      shooterSubsystem.shooterUnblock();
    else if (shooterBlock)
      shooterSubsystem.shooterBlock();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
