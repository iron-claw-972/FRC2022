/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrive;
import frc.robot.ControllerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends SubsystemBase {
  
  TalonFX leftMotor = ControllerFactory.createTalonFX(kDrive.kLeftMotorPort);
  // TalonFX leftMotorPal = ControllerFactory.createTalonFX(kDrive.kLeftMotorPalPort);

  TalonFX rightMotor = ControllerFactory.createTalonFX(kDrive.kRightMotorPort);
  // TalonFX rightMotorPal = ControllerFactory.createTalonFX(kDrive.kRightMotorPalPort);

  public Drivetrain() {
    // leftMotorPal.follow(leftMotor);
    // rightMotorPal.folalow(rightMotor);

    // Inverting opposite sides of the drivetrain
    leftMotor.setInverted(true);
    // rightMotorPal.setInverted(true);

    // leftMotor.setNeutralMode(NeutralMode.Coast);
    // rightMotor.setNeutralMode(NeutralMode.Coast);


  }

  // double lowSensThrottle = 0.2;
  // double lowSensTurn = 0.4;
  // double highSensThrottle = 1;
  // double highSensTurn = 0.5;

  double lowSensThrottle = 0.2;
  double lowSensTurn = 0.4;
  double highSensThrottle = 1;
  double highSensTurn = 0.5;

  double sensThrottle = lowSensThrottle;
  double sensTurn = lowSensTurn;
  public void modSensitivity(){
    if (sensThrottle == highSensThrottle) {
      sensThrottle = lowSensThrottle;
      sensTurn = lowSensTurn;
      System.out.println("sensitivity changed to low");
    } else {
      sensThrottle = highSensThrottle;
      sensTurn = highSensTurn;
      System.out.println("sensitivity changed to high");
    }
  }

  public void arcadeDrive(double throttle, double turn) {
    // System.out.println("arcade drive");
    leftMotor.set(ControlMode.PercentOutput, (throttle * sensThrottle + turn * sensTurn));
    rightMotor.set(ControlMode.PercentOutput, (throttle * sensThrottle - turn * sensTurn));
  }

  public void tankDrive(double left, double right) {
    leftMotor.set(ControlMode.PercentOutput, left);
    rightMotor.set(ControlMode.PercentOutput, right);
  }

  public void setEncoders(double left, double right) {
    leftMotor.setSelectedSensorPosition(left);
    rightMotor.setSelectedSensorPosition(right);
  } 

  public double getLeftEncoder() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightMotor.getSelectedSensorPosition();
  }

  public double getLeftVelocity() {
    return leftMotor.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return rightMotor.getSelectedSensorVelocity();
  }
  
  
  //dev zone below / experimental

  public void propDrive(double throttle, double turn){
    throttle = throttle * sensThrottle;
    turn = turn * sensTurn;
    
    double leftOut =throttle * (1 + turn);
    double rightOut=throttle * (1 - turn);

    leftMotor.set(ControlMode.PercentOutput, leftOut);
    rightMotor.set(ControlMode.PercentOutput, rightOut);
  }

  public void shiftDrive(double throttle, double turn) {
    
    throttle = throttle * sensThrottle;
    turn = turn * sensTurn;

    System.out.println("throttle: " + throttle);
    System.out.println("turn: " + turn);

    double leftOut =throttle;
    double rightOut=throttle;
    
    if (turn > 0){
      leftOut = leftOut + turn;
    } else if (turn < 0){
      rightOut = rightOut - turn;
    }

    if (leftOut > 1){
      rightOut = rightOut - (leftOut - 1);
      leftOut = 1;
    }
    if (rightOut > 1){
      leftOut = leftOut - (rightOut - 1);
      rightOut = 1;
    }    

    System.out.println("left: " + leftOut);
    System.out.println("Right: " + rightOut);

    leftMotor.set(ControlMode.PercentOutput, leftOut);
    rightMotor.set(ControlMode.PercentOutput, rightOut);
  }


  public double expoMS(double base, double exponent){
    //weird stuff will hapen if you don't put a number > 0
    double finVal = Math.pow(Math.abs(base),exponent);
    if (base < 0) {
      finVal *= -1;
    }
    return finVal;
  }

  String driveMode = "arcade";
  public void modDrive(){
    System.out.println("modding                         drive");
    if (driveMode == "arcade") {
      driveMode = "prop";
    }else if (driveMode == "prop") {
      driveMode = "arcade";
    }
  }
  public boolean isArcade(){
    return (driveMode == "arcade");
  }

  public void runDrive(double throttle, double turn){
    if (driveMode == "arcade") {
      this.arcadeDrive(throttle, turn);
    }if (driveMode == "shift") {
      this.shiftDrive(throttle, turn);
    }if (driveMode == "prop") {
      this.propDrive(throttle, turn);
    }
  }
  

}
