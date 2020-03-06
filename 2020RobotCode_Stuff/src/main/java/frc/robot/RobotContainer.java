/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.examples.ExampleCommand;
import frc.robot.commands.foldTheIntake;
import frc.robot.commands.intakingBall;
import frc.robot.commands.joystickControl;
import frc.robot.commands.moveBalls;
//import frc.robot.commands.panelSpinning;
import frc.robot.commands.shiftDrive;
import frc.robot.commands.shootBalls;
import frc.robot.commands.testing;
import frc.robot.commands.winchClimber;
import frc.robot.examples.ExampleSubsystem;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ballFeeder;
import frc.robot.subsystems.climbingWinch;
//import frc.robot.subsystems.controlPanel;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.flyWheel;
import frc.robot.subsystems.intakeFolder;
import frc.robot.subsystems.intakeOperator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;





/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

 
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //example command and subsystem
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //definition of all robot subsystems
  public static driveTrain driveTrain;
  public static intakeFolder intakeFolder;
  public static intakeOperator intakeOperator;
  public static LEDLights ledlights;
  public static flyWheel flyWheel;
  public static ballFeeder ballFeeder;
  public static climbingWinch climbingWinch;
  //public static controlPanel controlPanel;
  public static Limelight limelight;

  //definition of all robot joysticks
  public static Joystick driverJoystick;
  public static Joystick operatorJoystick;

  //definition of all robot buttons
  public static JoystickButton shiftButton;
  public static JoystickButton intakeFolderButtonReverse;
  public static JoystickButton intakeFolderButtonForward;
  public static JoystickButton intakeIn;
  public static JoystickButton intakeOut;
  public static JoystickButton shootBallsOut;
  public static JoystickButton shootBallsIn;
  public static JoystickButton moveBallsIn;
  public static JoystickButton moveBallsOut;
  public static JoystickButton climbWinchForward;
  public static JoystickButton climbWinchReverse;
  public static JoystickButton controlPanelClockWise;
  public static JoystickButton controlPanelCounterClockWise;
  

  public static JoystickButton lookForLEDButton;




  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  
  public RobotContainer() {

    //creating new instantiations of subsystems here
    driveTrain = new driveTrain();
    intakeFolder = new intakeFolder();
    intakeOperator = new intakeOperator();
    ledlights = new LEDLights();
    flyWheel = new flyWheel();
    ballFeeder = new ballFeeder();
    climbingWinch = new climbingWinch();
    //controlPanel = new controlPanel();
    limelight = new Limelight();

    //configure button bindings on controllers
    configureButtonBindings();
  }





  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */


  private void configureButtonBindings() {
    //instantiations of joysticks
    driverJoystick = new Joystick(Constants.driverJoystick);
    operatorJoystick = new Joystick(Constants.operatorJoystick);

    //default command of drivetrain
    driveTrain.setDefaultCommand(new joystickControl());

    //shifting button code
    shiftButton = new JoystickButton(driverJoystick, Constants.shiftingButton);
    shiftButton.toggleWhenPressed(new shiftDrive());

    //folding the intake button code (reverse)
    intakeFolderButtonReverse = new JoystickButton(operatorJoystick, Constants.intakeFolderButtonReverse);
    intakeFolderButtonReverse.whenPressed(new foldTheIntake());

    //folding the intake button code (forward)
    intakeFolderButtonForward = new JoystickButton(operatorJoystick, Constants.intakeFolderButtonForward);
    intakeFolderButtonForward.whenPressed(new foldTheIntake());

    //pull balls in
    intakeIn = new JoystickButton(driverJoystick, Constants.intakeInButton);
    intakeIn.whenPressed(new intakingBall());

    intakeOut = new JoystickButton(driverJoystick, Constants.intakeOutButton);
    intakeOut.whenPressed(new intakingBall());

    lookForLEDButton = new JoystickButton(operatorJoystick, Constants.controlPanelLEDSignal);
    lookForLEDButton.whenPressed(new testing());

    shootBallsOut = new JoystickButton(operatorJoystick, Constants.shootBallOut);
    shootBallsOut.whenPressed(new shootBalls());

    shootBallsIn = new JoystickButton(operatorJoystick, Constants.shootBallIn);
    shootBallsIn.whenPressed(new shootBalls());

    moveBallsIn = new JoystickButton(operatorJoystick, Constants.moveBallsIn);
    moveBallsIn.whenPressed(new moveBalls());

    moveBallsOut = new JoystickButton(operatorJoystick, Constants.moveBallsOut);
    moveBallsOut.whenPressed(new moveBalls());

    climbWinchForward = new JoystickButton(driverJoystick, Constants.climbWinchForward);
    climbWinchForward.whenPressed(new winchClimber());

    climbWinchReverse = new JoystickButton(driverJoystick, Constants.climbWinchReverse);
    climbWinchReverse.whenPressed(new winchClimber());

    //controlPanelClockWise = new JoystickButton(driverJoystick, Constants.controlPanelClockWise);
    //controlPanelClockWise.whenPressed(new panelSpinning());

    //controlPanelCounterClockWise = new JoystickButton(driverJoystick, Constants.controlPanelCounterClockWise);
    //controlPanelCounterClockWise.whenPressed(new panelSpinning());
  }





  //code which determines if joystick is pushed in the y direction (move forward, back)
  public static double getDriverYSpeed(){
    if (Math.abs(driverJoystick.getRawAxis(0)) >= 0.2){
      return (driverJoystick.getRawAxis(0));
    }
    return 0.0;
  }


  //code which determines if joystick is pushed in the x direction (turn left, right)
  public static double getDriverXSpeed(){
    if (Math.abs(driverJoystick.getRawAxis(1)) >= 0.2){
      return (driverJoystick.getRawAxis(1));
    }
    return 0.0;
  }





    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  //example autonomous command
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }





	public static void ledlights() {
	}
}
