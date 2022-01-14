/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.ControllerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends SubsystemBase {
  
  TalonFX leftMotor = ControllerFactory.createTalonFX(DriveConstants.kLeftMotorPort);
  TalonFX leftMotorPal = ControllerFactory.createTalonFX(DriveConstants.kLeftMotorPalPort);

  TalonFX rightMotor = ControllerFactory.createTalonFX(DriveConstants.kRightMotorPort);
  TalonFX rightMotorPal = ControllerFactory.createTalonFX(DriveConstants.kRightMotorPalPort);

  public Drivetrain() {
    leftMotorPal.follow(leftMotor);
    rightMotorPal.follow(rightMotor);

    // Inverting opposite sides of the drivetrain
    rightMotor.setInverted(true);
    rightMotorPal.setInverted(true);
  }

  public void arcadeDrive(double throttle, double turn) {
    leftMotor.set(ControlMode.PercentOutput, throttle + turn);
    rightMotor.set(ControlMode.PercentOutput, throttle - turn);
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
}
