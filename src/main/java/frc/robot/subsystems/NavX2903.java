package frc.robot.subsystems;

import frc.robot.Robot;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Super intense gyro thing
 */
public class NavX2903 {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean collisionDetected = false;
  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;
  private AHRS ahrs;
  
  final static double kCollisionThreshold_DeltaG = 0.5f;

  public NavX2903() {
    ahrs = new AHRS(Port.kMXP); 
    ScheduledThreadPoolExecutor exec = new ScheduledThreadPoolExecutor(1);
    exec.scheduleAtFixedRate(new Runnable() {
           public void run() {
                //Robot.sensorTable.getEntry("yaw").setDouble(turnAngle());
           }
       }, 0, 1000/25, TimeUnit.MILLISECONDS); // execute every 60 seconds
  }

  public void zero() {
    ahrs.zeroYaw();
  }

  public double turnAngle() {
    return ahrs.getAngle();
  }

  public void setBackwards(boolean isBackwards){
    if (isBackwards) ahrs.setAngleAdjustment(180);
  }

  public boolean isColliding() {
    collisionDetected = false;
    double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
        || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
      collisionDetected = true;
    }
    return collisionDetected;
  }

}
