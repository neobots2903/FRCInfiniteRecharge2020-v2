//Climber - Top of robot, on both sides. Starts within robot frame, can extend up and then further outwards, clips to bar, and pulls down to lift robot.
//Hardware involved:
//- One pneumatic solenoid: for raising and lowering climb arms (MUST NOT lower arms if robot is climbing)
//- One larger pneumatic solenoid: for reaching towards bar and pulling robot up. (MUST NOT extend if smaller solenoids are closed)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotMap;

public class Climb2903 {

    public Solenoid armRiserOpen;
    public Solenoid armRiserClose;
    public Solenoid armExtendOpen;
    public Solenoid armExtendClose;
    

    public Climb2903() {
        armRiserOpen = new Solenoid(RobotMap.armRiserOpen);
        armRiserClose = new Solenoid(RobotMap.armRiserClose);
        armExtendOpen = new Solenoid(RobotMap.armExtendOpen);
        armExtendClose = new Solenoid(RobotMap.armExtendClose);
        RetractArms();
        LowerArms();
    }

    public void RaiseArms() {
        armRiserOpen.set(true);
        armRiserClose.set(false);
    }

    public void LowerArms() {
        armRiserOpen.set(false);
        armRiserClose.set(true);
    }

    public void ExtendArms() {
        armExtendOpen.set(true);
        armExtendClose.set(false);
    }

    public void RetractArms() {
        armExtendOpen.set(false);
        armExtendClose.set(true);
    }

}