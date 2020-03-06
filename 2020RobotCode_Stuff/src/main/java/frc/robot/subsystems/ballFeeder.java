/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.moveBalls;


public class ballFeeder extends SubsystemBase {

  //definition of Spark MAX
  private CANSparkMax ballFeederMotor;





  public ballFeeder() {
    //instantiation of motor controller
    ballFeederMotor = new CANSparkMax(Constants.feederFlywheel, MotorType.kBrushless);

    //sets factory defaults of motor controller
    ballFeederMotor.restoreFactoryDefaults();
    ballFeederMotor.setIdleMode(IdleMode.kBrake);


  }


  //function which sets speed of motor if button is held
  public void pullBallsIn(){
      ballFeederMotor.set(-1.0);
  }





  //function which sets speed of motor forward if button is held
  public void pushBallsOut() {
      ballFeederMotor.set(1.0);
  }
  //function which sets speed of motor if button is not held
  public void stopMovingBalls() {
      ballFeederMotor.set(0);
  }





  //sets the initial command of the subsystem (found under commands folder)
  protected void initDefaultCommand() {
      setDefaultCommand(new moveBalls());
  }




  
  @Override
  public void periodic() {
    //System.out.println("Voltage: " + ballFeederMotor.getBusVoltage());
    //System.out.println("Temperature: " + ballFeederMotor.getMotorTemperature());
    //System.out.println("Output: " + ballFeederMotor.getAppliedOutput());
  }
}
