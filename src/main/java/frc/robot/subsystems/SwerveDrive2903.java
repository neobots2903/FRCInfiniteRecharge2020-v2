package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SwerveDrive2903 {

    public SwerveModule2903 LeftFront;
    public SwerveModule2903 RightFront;
    public SwerveModule2903 RightRear;
    public SwerveModule2903 LeftRear;

    public List<SwerveModule2903> modules = new ArrayList<SwerveModule2903>();
    int targetTicks = 0;

    final int TURN_TICKS_PER_REV = 4096 * 6;
    final int DEG_PER_REV = 360;
    boolean isForward = true;

    double joyDeadzone = 0.1; // joystick isn't actually in center, making sure doesn't move when not touched
                              // :)
    double triggerDeadzone = 0.1;
    int targetAngle = 0;

    public SwerveDrive2903() {
        LeftFront = new SwerveModule2903(RobotMap.LeftFrontForward, RobotMap.LeftFrontTurn, RobotMap.LeftFrontLimit);
        RightFront = new SwerveModule2903(RobotMap.RightFrontForward, RobotMap.RightFrontTurn,
                RobotMap.RightFrontLimit);
        RightRear = new SwerveModule2903(RobotMap.RightRearForward, RobotMap.RightRearTurn, RobotMap.RightRearLimit);
        LeftRear = new SwerveModule2903(RobotMap.LeftRearForward, RobotMap.LeftRearTurn, RobotMap.LeftRearLimit);

        LeftFront.setTurnDegreeOffset(225);
        RightFront.setTurnDegreeOffset(315);
        RightRear.setTurnDegreeOffset(45);
        LeftRear.setTurnDegreeOffset(135);

        modules.add(LeftFront);
        modules.add(RightFront);
        modules.add(RightRear);
        modules.add(LeftRear);
    }

    public void zeroModulesLimit() {
        SmartDashboard.putBoolean("Zeroin'", true);

        ArrayList<Thread> threads = new ArrayList<Thread>();
        for (SwerveModule2903 module : modules) {
            threads.add(new Thread(() -> {
                module.zeroTurnMotor();
            }));
        }
        for (Thread thread : threads) {
            thread.start();
        }
        for (Thread thread : threads) {
            try {
                thread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        isForward = true;
        SmartDashboard.putBoolean("Zeroin'", false);
        targetTicks = 0;
        targetAngle = 0;
        for (SwerveModule2903 module : modules) {
            module.TurnMotor.set(ControlMode.Position, 0);
        }
    }

    public void zeroModules() {
        targetTicks = 0;
        targetAngle = 0;
        for (SwerveModule2903 module : modules) {
            module.setEncoder(0);
            module.TurnMotor.set(ControlMode.Position, 0);
        }
    }

    public void goToZero() {
        setTurnDegrees(0);
    }

    public int joystickAngle(double x, double y) {
        int angle = -1;
        if (Math.abs(x) > joyDeadzone || Math.abs(y) > joyDeadzone) {
            angle = (int) Math.toDegrees(Math.atan2(x, y));
            if (angle < 0)
                angle += 360;
        }
        return angle;
    }

    public double joystickMag(double x, double y) {
        return Math.abs(Math.sqrt(x * x + y * y));
    }

    public int angleToTicks(int angle) {
        double remainder = angle;// % DEG_PER_REV;
        remainder /= DEG_PER_REV;
        return (int) (remainder * TURN_TICKS_PER_REV);
    }

    public void setForward(double speed) {
        for (SwerveModule2903 module : modules)
            module.ForwardMotor.set(speed * (isForward ? 1 : -1));
    }

    int loopCount = 0;

    public void setTurnDegrees(int degrees) {
        if (degrees == -1)
            return;
        int localTic = angleToTicks(degrees)
                - (targetTicks % TURN_TICKS_PER_REV + (targetTicks < 0 ? TURN_TICKS_PER_REV : 0));
        loopCount++;

        if (localTic < -TURN_TICKS_PER_REV / 2)
            localTic += TURN_TICKS_PER_REV;
        else if (localTic > TURN_TICKS_PER_REV / 2)
            localTic -= TURN_TICKS_PER_REV;

        if (Math.abs(localTic) > TURN_TICKS_PER_REV / 4) {
            isForward = !isForward;
            targetTicks += TURN_TICKS_PER_REV / 2 * (isForward ? 1 : -1);
            for (SwerveModule2903 module : modules) {
                module.setEncoder(targetTicks + module.getLastJoyTurnTicks());
                module.TurnMotor.set(targetTicks + module.getLastJoyTurnTicks());
            }
            if (loopCount > 4)
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            setTurnDegrees(degrees);
            return;
        }

        targetTicks += localTic;
        loopCount = 0;

        for (SwerveModule2903 module : modules)
            module.TurnMotor.set(ControlMode.Position,targetTicks + module.getJoyTurnTicks(degrees));
    }

    public void stopDrive() {
        setForward(0);
        for (SwerveModule2903 module : modules) {
            module.TurnMotor.set(ControlMode.PercentOutput,0);
            module.ForwardMotor.set(0);
        }
    }

    public double cleanAngle(double degrees) {
        return (degrees % DEG_PER_REV + (degrees < 0 ? DEG_PER_REV : 0));
    }

    public void swerveDrive(double power, double angle, double turn, boolean fieldCentric) {

        for (SwerveModule2903 module : modules)
            module.setJoyTurnPercent(turn);
        
        if (angle != -1) 
            targetAngle = (int)angle;
        double navXAngle = Robot.navXSubsystem.turnAngle();
        navXAngle = cleanAngle(navXAngle);
        setTurnDegrees((int)cleanAngle(targetAngle-((fieldCentric) ? navXAngle:0)));

        if (triggerDeadzone < Math.abs(power)) {
            setForward(power);
        }
    }
    
    public void swerveDriveDistance(double power, double angle, boolean fieldCentric, double distance){
        LeftFront.zeroForwardEncoder();
        RightRear.zeroForwardEncoder();
        while((LeftFront.getForwardMeters() + RightRear.getForwardMeters())/2 < distance){
            swerveDrive(power, angle, 0, fieldCentric);
        }
        swerveDrive(0, angle, 0, fieldCentric);
    }

    public void TankDrive(double left, double right) {
        goToZero();
        LeftFront.ForwardMotor.set(left);
        LeftRear.ForwardMotor.set(left);
        RightRear.ForwardMotor.set(right);
        RightFront.ForwardMotor.set(right);
    }

    public void ArcadeDrive(double forward, double turn) {
        TankDrive(forward+turn,forward-turn);
    }

}