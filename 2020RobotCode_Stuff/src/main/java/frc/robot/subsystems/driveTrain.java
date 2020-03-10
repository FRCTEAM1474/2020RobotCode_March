/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.states.shiftingGearboxStates;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;


public class driveTrain extends SubsystemBase {

  //definition of all Drivetrain Talons
  private final WPI_TalonSRX leftMaster;
  private final WPI_TalonSRX leftFollowerOne;
  private final WPI_TalonSRX leftFollowerTwo;
  private final WPI_TalonSRX rightMaster;
  private final WPI_TalonSRX rightFollowerOne;
  private final WPI_TalonSRX rightFollowerTwo;

  //definition of SpeedControllerGroups / DifferentialDrive in order to operate as one Gearbox / system
  private final SpeedControllerGroup leftMotorSpeedController;
  private final SpeedControllerGroup rightMotorSpeedController;
  public DifferentialDrive robotDrive;

  //definition of Pneumatics w/ Drivetrain
  public DoubleSolenoid shiftingSolenoid;
  public shiftingGearboxStates shiftState;

  //definitions for encoders
  public Faults encoderFaults;

  //gyro stuff
  public AHRS ahrs;

  public DifferentialDriveOdometry driveOdometry;
  public double vLeft, vRight, omegaLeft, omegaRight;

  public double distancePulse;

  public double leftTravel, rightTravel, leftPos, rightPos;

  
  public driveTrain() {

    //instantiations of Pneumatics
    shiftingSolenoid = new DoubleSolenoid(Constants.shiftingGearboxesOne, Constants.shiftingGearboxesTwo);
    shiftState = shiftingGearboxStates.IN;

    //LEFT GEARBOX
      //instantiations of Left Gearbox Motor Controllers
      leftMaster = new WPI_TalonSRX(Constants.leftMaster);
      leftFollowerOne = new WPI_TalonSRX(Constants.leftFollowerOne);
      leftFollowerTwo = new WPI_TalonSRX(Constants.leftFollowerTwo);

      //set other two motor controllers to follow master
      leftFollowerOne.follow(leftMaster);
      leftFollowerTwo.follow(leftMaster);
      leftMaster.setNeutralMode(NeutralMode.Brake);

      //sets all three motors controllers to factory defaults
      leftMaster.configFactoryDefault();
      leftFollowerOne.configFactoryDefault();
      leftFollowerTwo.configFactoryDefault();

      //creates a speed controller group to treat it as one "gearbox"
      leftMotorSpeedController = new SpeedControllerGroup(leftMaster, leftFollowerOne, leftFollowerTwo);

      //left master encoder code stuff
      encoderFaults = new Faults();

      leftMaster.setInverted(false);
      leftMaster.setSensorPhase(false);


    


    //RIGHT GEARBOX
      //instantiations of Right Gearbox Motor Controllers
      rightMaster = new WPI_TalonSRX(Constants.rightMaster);
      rightFollowerOne = new WPI_TalonSRX(Constants.rightFollowerOne);
      rightFollowerTwo = new WPI_TalonSRX(Constants.rightFollowerTwo);

      //set other two motor controllers to follow master
      rightFollowerOne.follow(rightMaster);
      rightFollowerTwo.follow(rightMaster);
      rightMaster.setNeutralMode(NeutralMode.Brake);
        
      //sets all three motors controllers to factory defaults
      rightMaster.configFactoryDefault();
      rightFollowerOne.configFactoryDefault();
      rightFollowerTwo.configFactoryDefault();

      //creates a speed controller group to treat it as one "gearbox"
      rightMotorSpeedController = new SpeedControllerGroup(rightMaster, rightFollowerOne, rightFollowerTwo);

      rightMaster.setInverted(false);
      rightMaster.setSensorPhase(false);





    //instantiation of new variable, which makes a drivetrain with both gearbox's code
    robotDrive = new DifferentialDrive(leftMotorSpeedController, rightMotorSpeedController);

    robotDrive.setSafetyEnabled(false);
        
    //System.out.println("Hello");

    ahrs = new AHRS(SPI.Port.kMXP);

    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    resetEncoders();

    leftPos = leftMaster.getSelectedSensorPosition();
    rightPos = rightMaster.getSelectedSensorPosition();

    leftTravel = leftPos * distancePerPulse();
    rightTravel = rightPos * distancePerPulse();

  }




  //function which switches pneumatic cylinders flow of air (changes gears)
  public void actuateShiftingGearboxes() {
    if (shiftState.equals(shiftingGearboxStates.IN)){
      shiftingSolenoid.set(Value.kReverse);
      shiftState = shiftingGearboxStates.OUT;
    }
    else {
      shiftingSolenoid.set(Value.kForward);
      shiftState = shiftingGearboxStates.IN;
    }
  }

  public double distancePerPulse() {
    distancePulse = (2 * Math.PI * Constants.wheelRadiusDrive) / (Constants.kSensorUnitsPerRotation * 3);

    return distancePulse;
  }

  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }


  public double getWheelSpeedLeft() {
    omegaLeft = ((leftMaster.getSelectedSensorVelocity() * Constants.kGearRatio) * 10 * ((Math.PI * 2) / Constants.kSensorUnitsPerRotation) * -1);
    vLeft = (omegaLeft * Constants.wheelRadiusDrive);

    return vLeft;
  }

  public double getWheelSpeedRight() {
    omegaRight = ((leftMaster.getSelectedSensorVelocity() * Constants.kGearRatio) * 10 * ((Math.PI * 2) / Constants.kSensorUnitsPerRotation) * -1);
    vRight = (omegaRight* Constants.wheelRadiusDrive);

    return vRight;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(vLeft, vRight);
  }


  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  //code which converts amount driver pushes joystick to output speed of motors. Also now used for autonomous turning when shooting
  public void driving(double speedL, double speedR){
    robotDrive.arcadeDrive(speedL, speedR);
  }

  public void driveWithVolts(double leftVolts, double rightVolts) {
    leftMotorSpeedController.setVoltage(leftVolts);
    rightMotorSpeedController.setVoltage(-rightVolts);
    robotDrive.feed();
  }

  public void resetEncoders() {
    leftPos = 0;
    rightPos = 0;
  }

  public double getAverageEncoderDistance() {
    return (leftTravel + rightTravel) / 2.0;
  }

  public void zeroHeading() {
    ahrs.zeroYaw();
  }

  public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getTurnRate() {
    return ahrs.getRate();
  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double ySpeed = RobotContainer.getDriverYSpeed();

    leftMaster.set(ControlMode.PercentOutput, ySpeed);
    leftMaster.getFaults(encoderFaults);

    rightMaster.set(ControlMode.PercentOutput, ySpeed);
    rightMaster.getFaults(encoderFaults);


    driveOdometry.update(Rotation2d.fromDegrees(getHeading()), leftTravel, rightTravel);
  }
}
