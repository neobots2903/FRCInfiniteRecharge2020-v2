/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    public static final int TBD = 0;
    //Servos
    public static final int intakeDropper = 1;
    public static final int shooterBlock = 0;

    //Motors
    public static final int shooterWheelL = 7;
    public static final int shooterWheelR = 6;
    public static final int shooterAngle = 40;
    public static final int intake = 37;

    public static final int colorWheel = TBD;

    public static final int LeftFrontForward = 23;
    public static final int LeftFrontTurn = 1;

    public static final int RightFrontForward = 20;
    public static final int RightFrontTurn = 2;
    public static final int RightRearForward = 21;
    public static final int RightRearTurn = 31;
    public static final int LeftRearForward = 22;
    public static final int LeftRearTurn = 38;

    //Pneumatics
    public static final int armExtendOpen = 2; 
    public static final int armExtendClose = 1; 
    public static final int armRiserOpen = 3;
    public static final int armRiserClose = 0;

   
    //Digital input output
    
    public static final int LidarLiteV3 = 0;
    public static final int LeftFrontLimit = 1;
    public static final int RightFrontLimit = 2;
    public static final int RightRearLimit = 3;
    public static final int LeftRearLimit = 4;
    public static final int shooterAngleTopLimit = 6;
    public static final int shooterAngleBottomLimit = 5;
    

    //Controllers
    public static final int driveJoy = 0;
    public static final int opJoy = 1;

    //I2C
    public static final int colorSensor = TBD;

    //Analog Input
    public static final int intakeDetect = TBD;
}