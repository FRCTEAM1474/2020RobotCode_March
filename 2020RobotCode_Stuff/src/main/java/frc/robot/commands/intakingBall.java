/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class intakingBall extends CommandBase 
{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  private boolean finished = false;



  public intakingBall() 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeOperator);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      if (RobotContainer.driverJoystick.getRawButton(Constants.intakeInButton)) 
      {
          RobotContainer.intakeOperator.pullBallIn();
      }
      else if (RobotContainer.driverJoystick.getRawButton(Constants.intakeOutButton)) 
      {
          RobotContainer.intakeOperator.pushBallOut();
      }
      else 
      {
          RobotContainer.intakeOperator.stopIntake();
      }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {

  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return finished;
  }
}
