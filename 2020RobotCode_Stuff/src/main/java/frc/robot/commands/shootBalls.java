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


public class shootBalls extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  private boolean finished = false;


  public shootBalls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.flyWheel);
    addRequirements(RobotContainer.ballFeeder);
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }





  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double steerCommand = 0.0;

      if (RobotContainer.operatorJoystick.getRawButton(Constants.shootBallIn)) {
          RobotContainer.flyWheel.shootBallsIn();
      }
      else if (RobotContainer.operatorJoystick.getRawButton(Constants.shootBallOut)) {
          RobotContainer.flyWheel.shootingTheBallsPID();
          Limelight.setPipeline(0);
          //RobotContainer.flyWheel.shootBallsIn();
          //RobotContainer.ballFeeder.pullBallsIn();
          //tx = RobotContainer.limelight.getTargetXAngle();

          final double STEER_K = 0.2;
        
          double tx = Limelight.getTargetXAngle();
          //double ta = Limelight.getTargetArea();
      
          double steer_cmd = tx * STEER_K;
          steerCommand = steer_cmd;

          if (tx != 0) {
            System.out.println("Hi. It's me again. I have a valid target.");
            // RobotContainer.driveTrain.robotDrive.arcadeDrive(drive, steerCommand);
            RobotContainer.driveTrain.auto_drive(0.2, steerCommand);
          }
          else if (tx < 2 && tx > -2){
            RobotContainer.driveTrain.auto_drive(0.0, 0.0);
          }
        }
      else {
          RobotContainer.flyWheel.stopShootingBalls();
          RobotContainer.flyWheel.limeLightStatus();
          RobotContainer.ballFeeder.stopMovingBalls();
      }
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
