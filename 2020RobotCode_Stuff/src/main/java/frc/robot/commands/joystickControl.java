/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class joystickControl extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  public double xSpeed, ySpeed;
  

  public joystickControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }





  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = RobotContainer.getDriverXSpeed();
    ySpeed = RobotContainer.getDriverYSpeed();
    RobotContainer.driveTrain.driving(xSpeed, ySpeed);

  }





  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }




  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
