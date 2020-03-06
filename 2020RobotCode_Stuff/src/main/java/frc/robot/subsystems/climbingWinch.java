/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.winchClimber;

import com.ctre.phoenix.motorcontrol.can.*;


public class climbingWinch extends SubsystemBase {

  //definition of all Drivetrain Talons
  private final WPI_TalonSRX climbMaster;
  private final WPI_VictorSPX climbFollower;

  private final SpeedControllerGroup climbGearbox;


  
  public climbingWinch() {

      climbMaster = new WPI_TalonSRX(Constants.climbMaster);
      climbFollower = new WPI_VictorSPX(Constants.climbFollower);
    
      climbFollower.follow(climbMaster);
    
      climbMaster.configFactoryDefault();
      climbFollower.configFactoryDefault();

      climbGearbox = new SpeedControllerGroup(climbMaster, climbFollower);

  }

  public void winchForward() {
      climbGearbox.set(1.0);
  }

  public void winchReverse() {
      climbGearbox.set(-1.0);
  }

  public void stopWinching() {
      climbGearbox.set(0.0);
  }

  //sets the initial command of the subsystem (found under commands folder)
  protected void initDefaultCommand() {
      setDefaultCommand(new winchClimber());
  }
}
