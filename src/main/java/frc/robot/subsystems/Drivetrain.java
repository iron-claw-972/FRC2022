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
  TalonFX leftMotorPal = ControllerFactory.createTalonFX(kDrive.kLeftMotorPalPort);

  TalonFX rightMotor = ControllerFactory.createTalonFX(kDrive.kRightMotorPort);
  TalonFX rightMotorPal = ControllerFactory.createTalonFX(kDrive.kRightMotorPalPort);

  public Drivetrain() {
    leftMotorPal.follow(leftMotor);
    rightMotorPal.follow(rightMotor);

    // Inverting opposite sides of the drivetrain
    rightMotor.setInverted(true);
    rightMotorPal.setInverted(true);
  }

  int sensitivity = 5;
  public void modSensitivity(){
    if (sensitivity == 5) {
      sensitivity = 2;
      System.out.println("sensitivity changed to 1/2");
    } else {
      sensitivity = 5;
      System.out.println("sensitivity changed to 1/5");
    }
  }

  public void arcadeDrive(double throttle, double turn) {
    leftMotor.set(ControlMode.PercentOutput, (throttle + turn) / sensitivity);
    rightMotor.set(ControlMode.PercentOutput, (throttle - turn) / sensitivity);
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
