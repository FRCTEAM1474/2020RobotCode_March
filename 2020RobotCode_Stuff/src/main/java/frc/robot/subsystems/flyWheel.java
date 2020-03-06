/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.swing.RootPaneContainer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.findDistance;
import frc.robot.commands.shootBalls;


public class flyWheel extends SubsystemBase {

  //definition of Spark MAX
  private CANSparkMax flyWheelMaster;
  private CANSparkMax flyWheelFollower;

  private final SpeedControllerGroup flyWheelGroup;

  private CANPIDController pidController, pidController_2;
  private CANEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private double steerCommand = 0.0;
  private boolean hasValidTarget = false;
  private double speedL, speedR;





  public flyWheel() {
    //instantiation of motor controller
    flyWheelMaster = new CANSparkMax(Constants.flyWheelMaster, MotorType.kBrushless);
    flyWheelFollower = new CANSparkMax(Constants.flyWheelFollower, MotorType.kBrushless);

    //sets factory defaults of motor controller
    flyWheelMaster.restoreFactoryDefaults();
    flyWheelMaster.setIdleMode(IdleMode.kBrake);

    flyWheelFollower.restoreFactoryDefaults();
    flyWheelFollower.setIdleMode(IdleMode.kBrake);
    flyWheelFollower.isFollower();
    flyWheelMaster.setInverted(true);

    flyWheelGroup = new SpeedControllerGroup(flyWheelMaster, flyWheelFollower);

    pidController = flyWheelMaster.getPIDController();
    pidController_2 = flyWheelFollower.getPIDController();
    encoder = flyWheelMaster.getEncoder();


    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    pidController_2.setP(kP);
    pidController_2.setI(kI);
    pidController_2.setD(kD);
    pidController_2.setIZone(kIz);
    pidController_2.setFF(kFF);
    pidController_2.setOutputRange(kMinOutput, kMaxOutput);
  }

  public double yAngleOffset() {
    double a2 = Limelight.getTargetYAngle();
    return a2;
  }

  public double targetDistance() {
    double distance = (Constants.h2 - Constants.h1)/(Math.tan((Constants.a1 + yAngleOffset())*2*Math.PI/360))/39.37;
    return distance;

  }  


  public double launchVelocity() {
    double deltaY = Constants.deltaY;
    double tangentAngle = Math.tan(Constants.setAngleBall*Math.PI/180);
    double cosineAngle = Math.cos(Constants.setAngleBall*Math.PI/180);
    double halfGravity = (Constants.gravity)/2;

    double xDistance = targetDistance();

    double launchVelocity = Math.sqrt(-1 * ((halfGravity) * ((xDistance)*(xDistance)) / (((deltaY - ((tangentAngle)*xDistance)) * ((cosineAngle)*(cosineAngle))))));

    return launchVelocity;
  }

  public double convertToRPM() {
    double rpm = ((launchVelocity() * 60) * 24 / (2 * Math.PI * Constants.radiusFlywheel * 15));

    return rpm;
  }

  public void shootingTheBallsPID() {
    double setPointOne, setPointTwo, velocityMotorOne, velocityMotorTwo;

    setPointOne = convertToRPM() * 5.7;  //constant multiplicative factor determined experimentally to consistently hit target
    setPointTwo = convertToRPM() * 7.75;

    if (targetDistance() < 4.5) {
      pidController.setReference(setPointOne, ControlType.kVelocity);
      pidController_2.setReference(setPointOne, ControlType.kVelocity);
    } else {
      pidController.setReference(setPointTwo, ControlType.kVelocity);
      pidController_2.setReference(setPointTwo, ControlType.kVelocity);
    }

    velocityMotorOne = encoder.getVelocity() * 4;
      if (velocityMotorOne >= setPointOne) {
        RobotContainer.ballFeeder.pullBallsIn();
      }
    velocityMotorTwo = encoder.getVelocity() * 2.8;
      if (velocityMotorTwo >= setPointTwo) {
        RobotContainer.ballFeeder.pullBallsIn();
      }

    
  }
  
  public void limeLightStatus() {
    if (Limelight.getPipeline() == 0 && RobotContainer.operatorJoystick.getRawButtonReleased(Constants.shootBallOut)) {
      Limelight.setPipeline(1);
    }
  }
  












  //function which sets speed of motor if button is held
  public void shootBallsIn(){
      flyWheelGroup.set(1);
  }





  //function which sets speed of motor forward if button is held
  public void shootBallsOut() {
      flyWheelGroup.set(-1);
  }
  //function which sets speed of motor if button is not held
  public void stopShootingBalls() {
      flyWheelGroup.set(0);
  }





  //sets the initial command of the subsystem (found under commands folder)
  protected void initDefaultCommand() {
      setDefaultCommand(new shootBalls());
  }




  
  @Override
  public void periodic() {
      //System.out.println(encoder.getVelocity());
      //System.out.println(convertToRPM() * 5.7);
    
   // updateLimeLightTracking();

    //double steer = RobotContainer.driverJoystick.getRawAxis(1);
    //double drive = RobotContainer.driverJoystick.getRawAxis(0);
    boolean auto = RobotContainer.operatorJoystick.getRawButtonPressed(1);

    if (auto) 
    {

      System.out.println(updateLimeLightTracking());
      //tx = RobotContainer.limelight.getTargetXAngle();
      if (updateLimeLightTracking() != 0) {
        System.out.println("Hi. It's me again. I have a valid target.");
       // RobotContainer.driveTrain.robotDrive.arcadeDrive(drive, steerCommand);
       RobotContainer.driveTrain.auto_drive(0.2, steerCommand);
      }
      else {
        RobotContainer.driveTrain.auto_drive(0.0, 0.0);
      }
    }
    else {
      RobotContainer.driveTrain.driving(speedL, speedR);
    }

  }

  public double updateLimeLightTracking() {
    final double STEER_K = 0.5;

    double tx = Limelight.getTargetXAngle();
    //double ta = Limelight.getTargetArea();

    hasValidTarget = true;

    double steer_cmd = tx * STEER_K;
    steerCommand = steer_cmd;

    return tx;
  }
  }
