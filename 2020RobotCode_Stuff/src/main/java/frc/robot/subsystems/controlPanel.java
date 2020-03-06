/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.panelSpinning;


public class controlPanel extends SubsystemBase {

  //definition of Spark MAX
  private CANSparkMax controlPanelMotor;





  public controlPanel() {
    //instantiation of motor controller
    controlPanelMotor = new CANSparkMax(Constants.controlPanel, MotorType.kBrushless);

    //sets factory defaults of motor controller
    controlPanelMotor.restoreFactoryDefaults();
    controlPanelMotor.setIdleMode(IdleMode.kBrake);


  }


  //function which sets speed of motor if button is held
  public void spinClockWise(){
      controlPanelMotor.set(1.0);
  }





  //function which sets speed of motor forward if button is held
  public void spinCounterClockWise() {
      controlPanelMotor.set(-1.0);
  }
  //function which sets speed of motor if button is not held
  public void stopSpinning() {
      controlPanelMotor.set(0);
  }





  //sets the initial command of the subsystem (found under commands folder)
  protected void initDefaultCommand() {
      setDefaultCommand(new panelSpinning());
  }




  
  @Override
  public void periodic() {
    //System.out.println("Voltage: " + controlPanelMotor.getBusVoltage());
    //System.out.println("Temperature: " + controlPanelMotor.getMotorTemperature());
    //System.out.println("Output: " + controlPanelMotor.getAppliedOutput());
  }
}*/
