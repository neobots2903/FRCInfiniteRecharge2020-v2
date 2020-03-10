package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class TeleOp2903 {

    boolean isFieldCentric = false;
    boolean isAutoAim = false;
    boolean driveAutoAimLock = false;
    boolean driveFieldCentricLock = false;
    boolean driveReZeroLock = false;

    double manualShootAngle = 45;
    double manualShootAngleLastTime = 0;
  
    public TeleOp2903() { 
    }

    public void initTeleOp() {
        Robot.shooterSubsystem.intakeOpen();
        Robot.climbSubsystem.RetractArms();
        Robot.climbSubsystem.LowerArms();
        Robot.shooterSubsystem.shooterBlock();
        Robot.shooterSubsystem.zeroShooterAngle();
        Robot.swerveDriveSubsystem.zeroModulesLimit();
    }
  
    public void runTeleOp() {
        double drivePower = Robot.driveJoy.getRawAxis(3) - Robot.driveJoy.getRawAxis(2); // Right Trigger - Left Trigger
        double driveAngle = Robot.swerveDriveSubsystem.joystickAngle(
            Robot.driveJoy.getRawAxis(0), 
            -Robot.driveJoy.getRawAxis(1)
        ); // Left Stick
        double driveTurn = Robot.driveJoy.getRawAxis(4); // Right Stick X
        boolean driveAutoAim = Robot.driveJoy.getRawButton(2); // B
        boolean driveFieldCentric = Robot.driveJoy.getRawButton(8); // Start
        boolean driveReZero = Robot.driveJoy.getRawButton(4); // Y

        double shooterSpeed = Robot.opJoy.getRawAxis(3) * Robot.shooterSubsystem.MAX_VEL; // Right Trigger
        double intakeSpeed = Robot.opJoy.getRawAxis(5) * 0.5; // Right Stick Y
        boolean shooterAuto = Robot.opJoy.getRawButton(1); // A
        boolean climbSafety = Robot.opJoy.getRawButton(8); // Start
        boolean climbRaise = Robot.opJoy.getRawButton(7); // Back
        boolean climbExtend = Robot.opJoy.getRawButton(6); // Right Bumper
        boolean climbRetract = Robot.opJoy.getRawButton(5); // Left Bumper
        boolean shooterUnblock = Robot.opJoy.getRawButton(3); // X
        boolean shooterBlock = Robot.opJoy.getRawButton(4); // Y
        boolean aimUp = Robot.opJoy.getPOV() == 0; //D-PAD Up
        boolean aimDown = Robot.opJoy.getPOV() == 180; //D-PAD Down

        if (driveAutoAim) {
            if (!driveAutoAimLock) {
                isAutoAim = !isAutoAim;
                driveAutoAimLock = true;
            }
        } else {
            driveAutoAimLock = false;
        }

        if (driveReZero) {
            if (!driveReZeroLock) {
                Robot.swerveDriveSubsystem.zeroModulesLimit();
                driveReZeroLock = true;
            }
        } else {
            driveReZeroLock = false;
        }

        if (driveFieldCentric) {
            if (!driveFieldCentricLock) {
                isFieldCentric = !isFieldCentric;
                driveFieldCentricLock = true;
            }
        } else {
            driveFieldCentricLock = false;
        }

        if (manualShootAngleLastTime + 50 < System.currentTimeMillis()) {
            if (aimUp && manualShootAngle < Robot.shooterSubsystem.MAX_SHOOT_ANGLE) {
                manualShootAngle += 0.5;
                manualShootAngleLastTime = System.currentTimeMillis();
            } else if (aimDown && manualShootAngle > 0) {
                manualShootAngle -= 0.5;
                manualShootAngleLastTime = System.currentTimeMillis();
            }
        }

        if (isAutoAim) {
            Robot.limelightSubsystem.setLight(true);
            Robot.shooterSubsystem.shooting(Robot.lidarSubsystem.getDistance(), 1, true);
            if (Robot.limelightSubsystem.getTV() == 1)
                driveTurn = MathUtil.clamp(Robot.visionTurn.calculate(Robot.limelightSubsystem.getTX(), 0), -0.8, 0.8);
        } else {
            Robot.limelightSubsystem.setLight(false);
        }

        Robot.swerveDriveSubsystem.swerveDrive(drivePower, driveAngle, driveTurn, isFieldCentric);

        if (shooterAuto) {
            Robot.shooterSubsystem.shooting(Robot.lidarSubsystem.getDistance(), 1, false);
        } else {
            if (shooterSpeed < 0.7) 
                Robot.shooterSubsystem.stopShoot();
            else 
                Robot.shooterSubsystem.shootSpeed(shooterSpeed);
            if (!isAutoAim) 
                Robot.shooterSubsystem.setAngle(manualShootAngle);
        }
        Robot.shooterSubsystem.intake(intakeSpeed);

        if (climbSafety) {
            if (climbRaise)
                Robot.climbSubsystem.RaiseArms();
            else if (climbExtend)
                Robot.climbSubsystem.ExtendArms();
            else if (climbRetract)
                Robot.climbSubsystem.RetractArms();
        }

        if (shooterUnblock)
            Robot.shooterSubsystem.shooterUnblock();
        else if (shooterBlock)
            Robot.shooterSubsystem.shooterBlock();

        SmartDashboard.putBoolean("Auto aim?", isAutoAim);
    }
}
