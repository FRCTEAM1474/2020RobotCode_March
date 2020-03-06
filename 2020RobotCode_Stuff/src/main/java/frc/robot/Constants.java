/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */





public class Constants {
    //joysticks
    public static int driverJoystick = 0;
    public static int operatorJoystick = 1;



    //Motor Controllers CAN ID's
        //Left Drivetrain Gearbox; (3) Talon SRX's
    public static int leftMaster = 0;
    public static int leftFollowerOne = 1;
    public static int leftFollowerTwo = 2;
    
        //Right Drivetrain Gearbox; (3) Talon SRX's
    public static int rightMaster = 3;
    public static int rightFollowerOne = 4;
    public static int rightFollowerTwo = 5;

        //Motor for Folding Intake; Spark MAX
    public static int intakeFolder = 6;

        //Motors for Running Flywheel; (2) Spark MAX's
    public static int flyWheelMaster = 10;
    public static int flyWheelFollower = 9;

        //Motors for Running Climb Gearbox; (2) Talon SRX's
    public static int climbMaster = 12;
    public static int climbFollower = 13;

        //Motor for Running Intake; Talon SRX
    public static int intakeOperator = 11;

        //Motor for Feeder to Flywheel; Spark MAX
    public static int feederFlywheel = 8;

        //Motor for Control Panel Motor; Spark MAX
    //public static int controlPanel = 9;



    //Buttons Driver Joystick
    public static int controlPanelLEDSignal = 12;           //operator joystick
    public static int shiftingButton = 5;                   //driver joystick
    public static int intakeFolderButtonReverse = 6;        //operator joystick
    public static int intakeFolderButtonForward = 4;        //operator joystick
    public static int intakeInButton = 1;                   //driver joystick
    public static int intakeOutButton = 2;                  //driver joystick
    public static int shootBallOut = 1;                     //operator joystick
    public static int shootBallIn = 2;                      //operator joystick
    public static int moveBallsIn = 5;                      //operator joystick
    public static int moveBallsOut = 3;                     //operator joystick
    public static int climbWinchReverse = 9;                //driver joystick
    public static int climbWinchForward = 10;               //driver joystick
    public static int controlPanelClockWise = 11;           //driver joystick
    public static int controlPanelCounterClockWise = 12;    //driver joystick

    

    //Solenoid port values
    public static int shiftingGearboxesOne = 0;
    public static int shiftingGearboxesTwo = 1;



    //encoder constants
    public static int kSensorUnitsPerRotation = 4096;
    public static int kGearRatio = 3;
    public static int kMaxRPM = 5310;

    public static int wheelRadius = 3;



    //intake folder constants
    public static float reverseLimit = 45;
    public static float forwardLimit = -45;
    public static float iDontKnow = -32;



    //PWM Constants
    public static int LEDMC = 1;








    //LimeLight Constants
    public static double h2 = 98.25;
    public static double h1 = 35;
    public static double a1 = 10.86;


    //PID Constants DriveTrain
    public static double kSDrive = 0.836;
    public static double kVDrive = 1.29;
    public static double kADrive = 0.268;
    public static double kPDrive = 0.000374;
    public static double kDDrive = 0.00018;


    //Flywheel Stuff
    public static double deltaY = 1.016;
    public static double setAngleBall = 26;
    public static double gravity = 9.81;
    public static double radiusFlywheel = .1524;
}
