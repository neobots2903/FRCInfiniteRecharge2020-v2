package frc.robot;

import edu.wpi.first.wpiutil.math.MathUtil;

public class Teleop2093 {

    boolean isFieldCentric = false;
    boolean driveFieldCentricLock = false;
    boolean driveReZeroLock = false;

    Robot robot;
  
    public Teleop2093(Robot robot) {
        this.robot = robot; 
    }
  
    public void runTeleOp() {
        double drivePower = robot.driveJoy.getRawAxis(3) - robot.driveJoy.getRawAxis(2); // Right Trigger - Left Trigger
        double driveAngle = robot.swerveDriveSubsystem.joystickAngle(robot.driveJoy.getRawAxis(0),
                robot.driveJoy.getRawAxis(1)); // Left Stick
        double driveTurn = robot.driveJoy.getRawAxis(4); // Right Stick X
        boolean driveAutoAim = robot.driveJoy.getRawButton(2); // B
        boolean driveFieldCentric = robot.driveJoy.getRawButton(8); // Start
        boolean driveReZero = robot.driveJoy.getRawButton(4); // Y

        double shooterSpeed = robot.opJoy.getRawAxis(3) * 10; // Right Trigger
        double intakeSpeed = robot.opJoy.getRawAxis(5); // Right Stick Y
        boolean shooterAuto = robot.opJoy.getRawButton(1); // A
        boolean climbSafety = robot.opJoy.getRawButton(8); // Start
        boolean climbRaise = robot.opJoy.getRawButton(7); // Back
        boolean climbExtend = robot.opJoy.getRawButton(6); // Right Bumper
        boolean climbRetract = robot.opJoy.getRawButton(5); // Left Bumper
        boolean shooterUnblock = robot.opJoy.getRawButton(3); // X
        boolean shooterBlock = robot.opJoy.getRawButton(4); // Y

        if (driveAutoAim && robot.limelightSubsystem.getTV() == 1) {
            robot.limelightSubsystem.setLight(true);
            driveTurn = MathUtil.clamp(robot.visionTurn.calculate(robot.limelightSubsystem.getTX(), 0), -1, 1);
        } else {
            robot.limelightSubsystem.setLight(false);
        }

        if (driveReZero) {
            if (!driveReZeroLock) {
                robot.swerveDriveSubsystem.zeroModulesLimit();
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

        robot.swerveDriveSubsystem.swerveDrive(drivePower, driveAngle, driveTurn, isFieldCentric);

        if (shooterAuto) {
            robot.shooterSubsystem.shooting(robot.lidarSubsystem.getDistance(), 1);
        } else {
            robot.shooterSubsystem.shootSpeed(shooterSpeed);
            robot.shooterSubsystem.setAngle(45);
        }
        robot.shooterSubsystem.intake(intakeSpeed);

        if (climbSafety) {
            if (climbRaise)
                robot.climbSubsystem.RaiseArms();
            else if (climbExtend)
                robot.climbSubsystem.ExtendArms();
            else if (climbRetract)
                robot.climbSubsystem.RetractArms();
        }

        if (shooterUnblock)
            robot.shooterSubsystem.shooterUnblock();
        else if (shooterBlock)
            robot.shooterSubsystem.shooterBlock();

    }
}
