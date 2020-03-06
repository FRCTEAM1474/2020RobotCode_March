/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LEDLights extends SubsystemBase {

  //definition of Spark MAX
  private static Spark LEDMC;

    public LEDLights() {
        // instantiation of motor controller
        LEDMC = new Spark(Constants.LEDMC);
    }

    // function which sets speed of motor if button is held
    public void setYellow() {
        LEDMC.set(0.67);
    }

    public void setRed() {
        LEDMC.set(0.59);
    }

    public void setBlue() {
        LEDMC.set(0.85);
    }

    public void setGreen() {
        LEDMC.set(0.75);
    }

    public void testing() {
        LEDMC.set(-0.45);
    }

    public void preMatchNotConnected() {
        LEDMC.set(-0.25);
    }

    public void preMatchConnected() {
        LEDMC.set(-0.21);
    }

    public void defaultMatch() {
        LEDMC.set(-0.87);
    }

    public void endGameWarning() {
        LEDMC.set(-0.11);
    }

  
  @Override
  public void periodic() {
    //System.out.println("Voltage: " + intakeFolderMotor.getBusVoltage());
    //System.out.println("Temperature: " + intakeFolderMotor.getMotorTemperature());
    //System.out.println("Output: " + intakeFolderMotor.getAppliedOutput());
  }
}
