/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class colorSensor extends SubsystemBase {

private final I2C.Port i2cPort;
private final ColorSensorV3 colorSensor;
public static ColorMatch colorMatcher;
public static Color kBlueTarget;
public static Color kGreenTarget;
public static Color kRedTarget;
public static Color kYellowTarget;
public static String colorString;

Color detectedColor;
ColorMatchResult match;

  
  public colorSensor() {
    
    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatcher = new ColorMatch();

    kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    detectedColor = colorSensor.getColor();
    match = colorMatcher.matchClosestColor(detectedColor);

  }

  public void determineColor() {

    if (match.color == kBlueTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Blue";
    }
    else if (match.color == kRedTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Red";
    }
    else if (match.color == kGreenTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Green";
    }
    else if (match.color == kYellowTarget && RobotContainer.operatorJoystick.getRawButton(Constants.controlPanelLEDSignal)) {
        colorString = "Yellow";
    }
    else {
        colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

  }



  public void setLEDLights() {
      if (colorString == "Blue") {
        RobotContainer.ledlights.setBlue();
      }
      else if (colorString == "Red") {
        RobotContainer.ledlights.setRed();
      }
      else if (colorString == "Green") {
        RobotContainer.ledlights.setGreen();
      }
      else if (colorString == "Yellow") {
        RobotContainer.ledlights.setYellow();
      }
  }



  public void stackedColorSensor() {
      determineColor();
      setLEDLights();
  }
}
