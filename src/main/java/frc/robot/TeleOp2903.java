package frc.robot;

import edu.wpi.first.wpiutil.math.MathUtil;

public class TeleOp2903 {

    boolean isFieldCentric = false;
    boolean driveFieldCentricLock = false;
    boolean driveReZeroLock = false;
  
    public TeleOp2903() { 
    }
  
    public void runTeleOp() {
        double drivePower = Robot.driveJoy.getRawAxis(3) - Robot.driveJoy.getRawAxis(2); // Right Trigger - Left Trigger
        double driveAngle = Robot.swerveDriveSubsystem.joystickAngle(
            Robot.driveJoy.getRawAxis(0), 
            Robot.driveJoy.getRawAxis(1)
        ); // Left Stick
        double driveTurn = Robot.driveJoy.getRawAxis(4); // Right Stick X
        boolean driveAutoAim = Robot.driveJoy.getRawButton(2); // B
        boolean driveFieldCentric = Robot.driveJoy.getRawButton(8); // Start
        boolean driveReZero = Robot.driveJoy.getRawButton(4); // Y

        double shooterSpeed = Robot.opJoy.getRawAxis(3) * 10; // Right Trigger
        double intakeSpeed = Robot.opJoy.getRawAxis(5); // Right Stick Y
        boolean shooterAuto = Robot.opJoy.getRawButton(1); // A
        boolean climbSafety = Robot.opJoy.getRawButton(8); // Start
        boolean climbRaise = Robot.opJoy.getRawButton(7); // Back
        boolean climbExtend = Robot.opJoy.getRawButton(6); // Right Bumper
        boolean climbRetract = Robot.opJoy.getRawButton(5); // Left Bumper
        boolean shooterUnblock = Robot.opJoy.getRawButton(3); // X
        boolean shooterBlock = Robot.opJoy.getRawButton(4); // Y

        if (driveAutoAim && Robot.limelightSubsystem.getTV() == 1) {
            Robot.limelightSubsystem.setLight(true);
            driveTurn = MathUtil.clamp(Robot.visionTurn.calculate(Robot.limelightSubsystem.getTX(), 0), -1, 1);
        } else {
            Robot.limelightSubsystem.setLight(false);
        }

        if (driveReZero) {
            if (!driveReZeroLock) {
                Robot.swerveDriveSubsystem.zeroModulesLimit();
                driveReZeroLock = true;
            }
        } else
            driveReZeroLock = false;

        if (driveFieldCentric) {
            if (!driveFieldCentricLock) {
                isFieldCentric = !isFieldCentric;
                driveFieldCentricLock = true;
            }
        } else
            driveFieldCentricLock = false;

            Robot.swerveDriveSubsystem.swerveDrive(drivePower, driveAngle, driveTurn, isFieldCentric);

        if (shooterAuto) {
            Robot.shooterSubsystem.shooting(Robot.lidarSubsystem.getDistance(), 1);
        } else {
            Robot.shooterSubsystem.shootSpeed(shooterSpeed);
            Robot.shooterSubsystem.setAngle(45);
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

    }
}
