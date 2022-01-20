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
import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends SubsystemBase {
  
  TalonFX leftMotor = ControllerFactory.createTalonFX(kDrive.kLeftMotorPort);
  // TalonFX leftMotorPal = ControllerFactory.createTalonFX(kDrive.kLeftMotorPalPort);

  TalonFX rightMotor = ControllerFactory.createTalonFX(kDrive.kRightMotorPort);
  // TalonFX rightMotorPal = ControllerFactory.createTalonFX(kDrive.kRightMotorPalPort);

  public Drivetrain() {
    // leftMotorPal.follow(leftMotor);
    // rightMotorPal.follow(rightMotor);

    // Inverting opposite sides of the drivetrain
    rightMotor.setInverted(true);
    // rightMotorPal.setInverted(true);
  }

  double lowSensThrottle = 0.2;
  double lowSensTurn = 0.4;
  double highSensThrottle = 0.5;
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
    leftMotor.set(ControlMode.PercentOutput, -(throttle * sensThrottle - turn* sensTurn));
    rightMotor.set(ControlMode.PercentOutput, (throttle * sensThrottle + turn* sensTurn));
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

  public void modArcadeDrive1(double throttle, double turn) {
    double leftOut =throttle;
    double rightOut=throttle;
    if (turn > 0){
      rightOut += turn;
    } else if (turn < 0){
      leftOut += -turn;
    }

    if (leftOut > 1){
      rightOut = rightOut - (leftOut - 1);
      leftOut = 1;
    } else if (rightOut > 1){
      leftOut = leftOut - (rightOut - 1);
      rightOut = 1;
    }    

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
}
