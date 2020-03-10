package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Shooter2903 {
    /**
     * Creates a new ExampleSubsystem.
     */

    final double POWER_CELL_WEIGHT = 0.142; // Kg
    final double POWER_CELL_DIAMETER = 17.78; // cm
    final double WHEEL_DIAMETER = 10.16; //cm
    final double ROBOT_SHOOTER_HEIGHT = 0.33782; // m
    final double GOAL_HEIGHT = 2.5 - ROBOT_SHOOTER_HEIGHT; // m
    final double VEL_MULT = 1.6;
    public final double MAX_VEL = 20/VEL_MULT; // m/s
    public final double MAX_SHOOT_ANGLE = 50; // degree
    final double GRAV = 9.80665; // m/s/s
    final double DEG_PER_REV = 360; // degrees per revolution
    final double TICKS_PER_REV = 4096; // ticks per revolution
    final double MAX_LIMIT_ANGLE = 57; // highest degrees possible
    final double PORT_DEPTH = 0.74295; // m
    final double SPEED_ERROR = 1.5; // m/s

    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    double lastSetSpeed = 0;

    WPI_TalonSRX shooterWheelL;
    WPI_TalonSRX shooterWheelR;
    WPI_TalonSRX shooterAngle;
    WPI_TalonSRX intake;
    Servo intakeDropper;
    Servo shooterBlock;
    DigitalInput shooterAngleTopLimit;
    DigitalInput shooterAngleBottomLimit;
    AnalogInput intakeDetect;

    public Shooter2903() {
        shooterWheelL = new WPI_TalonSRX(RobotMap.shooterWheelL);
        shooterWheelR = new WPI_TalonSRX(RobotMap.shooterWheelR);
        shooterAngle = new WPI_TalonSRX(RobotMap.shooterAngle);
        intake = new WPI_TalonSRX(RobotMap.intake);
        intakeDropper = new Servo(RobotMap.intakeDropper);
        shooterBlock = new Servo(RobotMap.shooterBlock);

        shooterAngleTopLimit = new DigitalInput(RobotMap.shooterAngleTopLimit);
        shooterAngleBottomLimit = new DigitalInput(RobotMap.shooterAngleBottomLimit);
        // intakeDetect = new AnalogInput(RobotMap.intakeDetect);
        shooterWheelL.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        shooterWheelR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        shooterAngle.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, kPIDLoopIdx, kTimeoutMs);
        
        shooterWheelL.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        shooterWheelR.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        shooterWheelL.configVelocityMeasurementWindow(32);
        shooterWheelR.configVelocityMeasurementWindow(32);

        shooterWheelL.config_kP(0, 0.14);
        shooterWheelR.config_kP(0, 0.14);
        shooterWheelL.config_kI(0, 0.001);
        shooterWheelR.config_kI(0, 0.001);
        shooterWheelL.config_kD(0, 0.5);
        shooterWheelR.config_kD(0, 0.5);
        shooterWheelL.config_kF(0, 1023.0/25600.0);
        shooterWheelR.config_kF(0, 1023.0/25600.0);

        shooterWheelL.configNominalOutputForward(0, kTimeoutMs);
		shooterWheelL.configNominalOutputReverse(0, kTimeoutMs);
		shooterWheelL.configPeakOutputForward(1, kTimeoutMs);
        shooterWheelL.configPeakOutputReverse(-1, kTimeoutMs);

        shooterWheelR.configNominalOutputForward(0, kTimeoutMs);
		shooterWheelR.configNominalOutputReverse(0, kTimeoutMs);
		shooterWheelR.configPeakOutputForward(1, kTimeoutMs);
        shooterWheelR.configPeakOutputReverse(-1, kTimeoutMs);
        
        shooterWheelL.setInverted(false);
        shooterWheelR.setInverted(true);

        shooterAngle.config_kP(0, 4);
        shooterAngle.setSelectedSensorPosition(0);
    }

    public void shooterBlock(){
        shooterBlock.set(1);
    }

    public void shooterUnblock(){
        shooterBlock.set(0);
    }

    public void intakeOpen() {
        intakeDropper.set(1);
    }

    public void intakeClose() {
        intakeDropper.set(0);
    }

    public boolean intakeDetect(){
        if((intakeDetect.getVoltage()/2)/2.54 < 8) return true; else return false;
    }

    public boolean getTop() {
        return !shooterAngleTopLimit.get();
    }

    public boolean getBottom() {
        return !shooterAngleBottomLimit.get();
    }

    public double getAngleCurrent() {
        return shooterAngle.getStatorCurrent();
    }

    public void zeroShooterAngle() {
        while (!getTop()) {
            shooterAngle.set(ControlMode.PercentOutput, 0.3);
        }
        shooterAngle.set(ControlMode.PercentOutput, 0);
        shooterAngle.setSelectedSensorPosition(convertAngleToTicks(MAX_LIMIT_ANGLE));
    }

    public void checkShootLimits() {
        if (getTop())
            if (shooterAngle.getMotorOutputVoltage() > 0) shooterAngle.setVoltage(0);
        if (getBottom())
            if (shooterAngle.getMotorOutputVoltage() < 0) shooterAngle.setVoltage(0);
    }

    public void shootSpeed(double metersPerSec) {
        metersPerSec *= VEL_MULT;
        lastSetSpeed = metersPerSec;
        double velocity = convertToTalonVelocity(metersPerSec); // calc power
        SmartDashboard.putNumber("Target shoot speed", metersPerSec);
        shooterWheelL.set(ControlMode.Velocity, velocity);
        shooterWheelR.set(ControlMode.Velocity, velocity);
    }

    public void stopShoot() {
        shooterWheelL.set(ControlMode.PercentOutput, 0);
        shooterWheelR.set(ControlMode.PercentOutput, 0);
    }

    public void waitForTargetSpeed() {
        while (Math.abs(lastSetSpeed - getCurrentSpeed()) > SPEED_ERROR) {
            try {
                wait(100);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public double getLeftSpeed() {
        return convertToMetersPerSec(shooterWheelL.getSelectedSensorVelocity());
    }

    public double getRightSpeed() {
        return convertToMetersPerSec(shooterWheelR.getSelectedSensorVelocity());
    }

    public double getCurrentSpeed(){
       return (getLeftSpeed() + getRightSpeed())/2;
    }

    /**
     * Configures shooter to correct angle and speed with distance in m
     * @param distance distance to the ports in m
     * @param timeCorrect standard is 1 || decrease value for faster shoot || it decreases 
     * time automatically when shoot is not possible with current time
     */
    public void shooting(double distance, double timeCorrect, boolean angleOnly){
        double[] data = shootMath(distance, timeCorrect);
        if(data[0] == -1) SmartDashboard.putString("Shoot Error:", "velocity is too big");
        if(data[1] == -1)SmartDashboard.putString("Shoot Error:", "angle is too big");
        if(data[0] >= 0 && data[1] >= 0){
            SmartDashboard.putString("Shoot Error:", "none");
            if (!angleOnly) shootSpeed(data[0]);
            setAngle(data[1]);
        } else {
            stopShoot();
        }
        
    }

    public void setAngle(double angle) {
        if (angle < 0) angle = 0;
        if (angle > MAX_SHOOT_ANGLE) angle = MAX_SHOOT_ANGLE;
        SmartDashboard.putNumber("Target shoot angle", angle);
        double ticks = convertAngleToTicks(angle);
        shooterAngle.set(ControlMode.Position,ticks);
    }

    public double getAngle(){
        return convertTicksToAngle(getAngleTicks());
    }

    public int getAngleTicks() {
        return shooterAngle.getSelectedSensorPosition();
    }

    public void intake(double power) {
        intake.set(ControlMode.PercentOutput,power);
    }

    public double tMax(double timeCorrect){
        double time = Math.sqrt((2*GOAL_HEIGHT)/GRAV);
        return time*timeCorrect;
    }

    public double VelInitX(double distance, double timeCorrect){
        double Vix = distance/tMax(timeCorrect);
        return Vix;
    }

    public double VelInitY(double timeCorrect){
        double Viy = (GOAL_HEIGHT + (0.5*GRAV*(Math.pow (tMax(timeCorrect), 2))))/tMax(timeCorrect);
        return Viy;
    }

    public double InitVel(double distance, double timeCorrect){
        double Vi = Math.sqrt(Math.pow(VelInitX(distance, timeCorrect), 2) + Math.pow(VelInitY(timeCorrect), 2));
        return Vi;
    }

    public double Angle(double distance, double timeCorrect){
        double angle = Math.atan(VelInitY(timeCorrect)/VelInitX(distance, timeCorrect))*(180/Math.PI);

        return angle;
    }

    public double[] shootMath(double distance, double timeCorrect){
        distance += PORT_DEPTH;
        double InitVel = InitVel(distance, timeCorrect);
        double Angle = Angle(distance, timeCorrect);
        for(double a = 1.5; Angle > MAX_SHOOT_ANGLE && InitVel < MAX_VEL+5; a+=0.25){
            InitVel = InitVel(distance, timeCorrect/a); //dividing by a makes bigger velocity and smaller angle 
            Angle = Angle(distance, timeCorrect/a);
        }
        double[] data = {InitVel, Angle};
        if(InitVel > MAX_VEL && InitVel < MAX_VEL+5) data[0] = MAX_VEL;
        if(InitVel > MAX_VEL+5) data[0] = -1;
        if(Angle > MAX_SHOOT_ANGLE) data[1] = -1;
        return data;
    }

    // public double shooterAngleMath(double distance, double vel) {
    //     double angle = 0;
    //     double mult = 0.95;
    //     angle = (Math.asin((distance * GRAV) / Math.pow(vel, 2))) / 2;// math formula without air ressitance
    //     // angle = angle * mult;// add multiplier
    //     if (angle < MAX_SHOOT_ANGLE)
    //         return angle; // looking if angle is lower than max angle
    //     else
    //         return -1; // -1 stands for error
    // }

    // public double shooterVelMath(double distance, double angle) {
    //     double vel = 0;
    //     double mult = 0.95;
    //     // math formula without air ressitance
    //     vel = Math.sqrt((distance * GRAV) / Math.sin(2 * angle));
    //     // add multiplier
    //     // vel = vel * mult;
    //     if (vel < MAX_VEL)
    //         return vel; // looking if velocity is lower max velocity
    //     else
    //         return -1; // -1 stands for error
    // }

    public int convertToTalonVelocity(double metersPerSec) {
        //distance wheel spins each revolution
        double circumference = Math.PI * WHEEL_DIAMETER/100;
        //ticks encoder counts per meter
        double ticksPerMeter = TICKS_PER_REV / circumference;
        //(m per sec) * (tick per m) = (tick per sec)
        double ticksPerSec = metersPerSec * ticksPerMeter;
        //(tick per sec) * (sec per 10 * tenth sec) = (tick per tenth sec)
        double ticksPerTenthSec = ticksPerSec / 10;
        return (int)ticksPerTenthSec;
    }

    public double convertToMetersPerSec(double talonVelocity) {
        double ticksPerTenthSec = talonVelocity;
        //(ticks per tenth sec) * 10 = (tick per sec)
        double ticksPerSec = ticksPerTenthSec*10;
        //distance wheel spins each revolution
        double circumference = Math.PI * WHEEL_DIAMETER/100;
        //meters per ticks encoder count
        double metersPerTick = circumference / TICKS_PER_REV;
        //(ticks per sec) * (meters per tick) = (meters per sec)
        double metersPerSecond = ticksPerSec * metersPerTick;
        return metersPerSecond;
    }

    public int convertAngleToTicks(double degrees) {
        double remainder = degrees;
        remainder /= DEG_PER_REV;
        return (int)(remainder * TICKS_PER_REV);
    }

    public int convertTicksToAngle(double ticks) {
        double remainder = ticks;
        remainder /= TICKS_PER_REV;
        return (int)(remainder * DEG_PER_REV);
    }

}