/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  // definition of auto, robot container, compressor

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static Compressor compressor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  public static ColorMatch colorMatcher = new ColorMatch();

  public static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public static String colorString;

  public static double timer;
  public static DriverStation FRC1474DriverStation;

  private static final double kHoldDistance = 12.0;

  // factor to convert sensor values to a distance in inches
  private static final double kValueToInches = 0.125;

  // proportional speed constant
  private static final double kP = 0.05;

  private static final int kUltrasonicPort = 0;

  private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    compressor = new Compressor();

    Limelight.setPipeline(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */

  @Override
  public void autonomousPeriodic() {

    
    timer = Timer.getMatchTime();

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    Color detectedColor = colorSensor.getColor();

    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Blue";
        RobotContainer.ledlights.setBlue();
      } else if (match.color == kRedTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Red";
        RobotContainer.ledlights.setRed();
      } else if (match.color == kGreenTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Green";
        RobotContainer.ledlights.setGreen();
      } else if (match.color == kYellowTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Yellow";
        RobotContainer.ledlights.setYellow();
      } else {
        colorString = "Unknown";
      }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("Ultrasonic", m_ultrasonic.getValue());




    
    timer = Timer.getMatchTime();

  }





  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.

    CommandScheduler.getInstance().cancelAll();
  }





  /**
   * This function is called periodically during test mode.
   */


  @Override
  public void testPeriodic() {
  }
}
