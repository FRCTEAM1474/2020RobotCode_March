/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.intakingBall;


public class intakeOperator extends SubsystemBase {

  //definition of Spark MAX
  private WPI_TalonSRX intakeOperator;





  public intakeOperator() {
    //instantiation of motor controller
    intakeOperator = new WPI_TalonSRX(Constants.intakeOperator);

    //sets factory defaults of motor controller
    intakeOperator.configFactoryDefault();
    intakeOperator.setInverted(false);

    
  }





  //function which sets speed of motor if button is held
  public void pullBallIn(){
      intakeOperator.set(0.75);
  }





  //function which sets speed of motor forward if button is held
  public void pushBallOut() {
      intakeOperator.set(-0.75);
  }



  //function which sets speed of motor if button is not held
  public void stopIntake() {
      intakeOperator.set(0);
  }





  //sets the initial command of the subsystem (found under commands folder)
  protected void initDefaultCommand() {
      setDefaultCommand(new intakingBall());
  }




  
  @Override
  public void periodic() {
    //System.out.println("Voltage: " + intakeFolderMotor.getBusVoltage());
    //System.out.println("Temperature: " + intakeFolderMotor.getMotorTemperature());
    //System.out.println("Output: " + intakeFolderMotor.getAppliedOutput());
  }
}
