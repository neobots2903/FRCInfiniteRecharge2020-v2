package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class SwerveModule2903 {
  public CANSparkMax ForwardMotor;
  public WPI_TalonSRX TurnMotor;
  public DigitalInput limit;
  final int TURN_TICKS_PER_REV = 4096 * 6;
  final int FORWARD_TICKS_PER_REV = 42;
  final int DEG_PER_REV = 360;
  final double FORWARD_WHEEL_DIAM = 0.1016; // m
  final double FORWARD_WHEEL_CIRC = Math.PI * FORWARD_WHEEL_DIAM; // m
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  private double turnDegPct = 0;
  private int turnDegOff = 0;
  private int lastTurnTicks = 0;

  public SwerveModule2903(int forwardMotorId, int turnMotorId, int limitId) {
    ForwardMotor = new SafeCANSparkMax(forwardMotorId, MotorType.kBrushless);
    TurnMotor = new WPI_TalonSRX(turnMotorId);
    limit = new DigitalInput(limitId);
    
    ForwardMotor.setInverted(false);
    ForwardMotor.setSmartCurrentLimit(40);

    TurnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);

    TurnMotor.configPeakCurrentLimit(45, 0);
    TurnMotor.configPeakCurrentDuration(750, 0);
    TurnMotor.configContinuousCurrentLimit(0, 0);
    TurnMotor.configVoltageCompSaturation(6);

    // setZero();
    setEncoder(0);

    setPowerPercent(1); // set max turn power to just 100%;
    // TurnMotor.set(ControlMode.Position, getTurnTicks());
    TurnMotor.config_kP(0, 0.75);
  }

  public boolean getLimit() {
    return limit.get();
  }

  public void zeroTurnMotor() {
    for (int i = 0; i < 2; i++) {
      if (i == 1) {
        try {
          Thread.sleep(400);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
      while (!getLimit())
        TurnMotor.set(ControlMode.PercentOutput, (i==0) ? 0.9 : -0.75);
      TurnMotor.set(ControlMode.PercentOutput, 0);
    }
    setEncoder(0);
  }

  public void setZero() {
    /**
     * Grab the 360 degree position of the MagEncoder's absolute
     * position, and intitally set the relative sensor to match.
     */
    int absolutePosition = TurnMotor.getSensorCollection().getPulseWidthPosition();
    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    setEncoder(absolutePosition);
  }

  public void setEncoder(int newPos) {
    TurnMotor.setSelectedSensorPosition(newPos, kPIDLoopIdx, kTimeoutMs);
  }

  public void setTurnDegreeOffset(int deg) {
    turnDegOff = deg;
  }

  public void setJoyTurnPercent(double percent) {
    turnDegPct = percent;
  }

  public int getLastJoyTurnTicks() {
    return lastTurnTicks;
  }

  public int getJoyTurnTicks(double degrees) {
    double calc = turnDegOff-degrees;
    if (turnDegPct > 0) calc += 180;
    if (calc > 180) calc -= 360;
    else if (calc < -180) calc += 360;
    lastTurnTicks = angleToTicks((int)(calc*Math.abs(turnDegPct)));
    return lastTurnTicks;
  }

  public int getTurnTicks() {
    return TurnMotor.getSensorCollection().getQuadraturePosition();// % TICKS_PER_REV;
  }

  public double getForwardTicks(){
    return ForwardMotor.getEncoder().getPosition();
  }

  public void zeroForwardEncoder(){
    ForwardMotor.getEncoder().setPosition(0);
  }

  public double getForwardMeters(){
    return (getForwardTicks()/FORWARD_TICKS_PER_REV)*FORWARD_WHEEL_CIRC;
  }

  public int getAbsoluteTurnTicks() {
    return TurnMotor.getSensorCollection().getPulseWidthPosition();// % TICKS_PER_REV;
  }

  public int getTurnDegrees() {
    return ticksToAngle(getTurnTicks());
  }

  public int getAbsoluteTurnDegrees() {
    return ticksToAngle(getTurnTicks()%TURN_TICKS_PER_REV);
  }

  public int ticksToAngle (int ticks) {
    double remainder = ticks;// % TICKS_PER_REV;
    remainder /= TURN_TICKS_PER_REV;
    return (int)(remainder * DEG_PER_REV);
  }

  public int angleToTicks (int angle) {
    double remainder = angle;// % DEG_PER_REV;
    remainder /= DEG_PER_REV;
    return (int)(remainder * TURN_TICKS_PER_REV);
  }

  public void setPowerPercent(double val) {
    double value = 
      (val > 1) ? 1 : 
      (val < 0) ? 0 : val;

    TurnMotor.configPeakOutputForward(value, 0);
    TurnMotor.configPeakOutputReverse(-value, 0);
  }

  private class SafeCANSparkMax extends CANSparkMax {
    private int safetyTimeout;
    private long endgame = System.currentTimeMillis();
    private boolean running = true;

    public SafeCANSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
        setSafetyTimeout(200);
        run();
    }

    @Override
    public void set(double speed) {
      super.set(speed);
      endgame = System.currentTimeMillis() + safetyTimeout;
    }

    // public void destroySafety() {
    //   running = false;
    // }

    public void setSafetyTimeout(int millis) {
      safetyTimeout = millis;
    }
 
    private void run() {
      new Thread(() -> {
        while (running) {
          try {
            if (endgame < System.currentTimeMillis()) {
              super.set(0);
            }
            Thread.sleep(10);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }).start();
    }
  }

}