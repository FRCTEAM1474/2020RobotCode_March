/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.foldTheIntake;


public class intakeFolder extends SubsystemBase {
  
  //definition of Spark MAX
  private CANSparkMax intakeFolderMotor;





  public intakeFolder() {
    //instantiation of motor controller
    intakeFolderMotor = new CANSparkMax(Constants.intakeFolder, MotorType.kBrushless);

    //sets factory defaults of motor controller
    intakeFolderMotor.restoreFactoryDefaults();
    intakeFolderMotor.setIdleMode(IdleMode.kBrake);

    intakeFolderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    intakeFolderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    double forwardLimitValue = intakeFolderMotor.getSoftLimit(SoftLimitDirection.kForward);

    if (forwardLimitValue > 0 || forwardLimitValue < 0) {
      forwardLimitValue = 0;

      intakeFolderMotor.setSoftLimit(SoftLimitDirection.kForward, (float) forwardLimitValue);
    }

    intakeFolderMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.iDontKnow);
  }





  //function which sets speed of motor if button is held
  public void driveTheIntakeReverse(){
      intakeFolderMotor.set(.3);
  }





  //function which sets speed of motor forward if button is held
  public void driveTheIntakeForward() {
      intakeFolderMotor.set(-.3);
  }
  //function which sets speed of motor if button is not held
  public void stopDrivingIntake() {
      intakeFolderMotor.set(0);
  }





  //sets the initial command of the subsystem (found under commands folder)
  protected void initDefaultCommand() {
      setDefaultCommand(new foldTheIntake());
  }




  
  @Override
  public void periodic() {
    //System.out.println("Voltage: " + intakeFolderMotor.getBusVoltage());
    //System.out.println("Temperature: " + intakeFolderMotor.getMotorTemperature());
    //System.out.println("Output: " + intakeFolderMotor.getAppliedOutput());
  }
}
