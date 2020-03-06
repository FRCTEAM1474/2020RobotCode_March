/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class findDistance extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  private boolean finished = false;


  public findDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.limelight);

  }

  public double yAngleOffset() {
    double a2 = Limelight.getTargetYAngle();
    return a2;
  }

  public double targetDistance() {
    double distance = (Constants.h2 - Constants.h1)/(Math.atan(Constants.a1 + yAngleOffset()));
    return distance;
  }
  
  private void addRequirements(Limelight limelight) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }





  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      targetDistance();
      System.out.println("The Target Distance is: " + targetDistance());
  }





  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }





  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
