/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * Drives a certain distance
 */
public class DriveRotate extends CommandBase {
  double setpoint, zeroPos;

  public DriveRotate(double rotatoion) {
    addRequirements(RobotContainer.m_drive);
    setpoint = rotatoion;
  }

  @Override
  public void initialize() {
    zeroPos = RobotContainer.navX.getAngle();
    RobotContainer.m_drive.setBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    RobotContainer.m_drive.tankDrive(
      Math.copySign(RobotContainer.driveConstants.kAutoDriveSpeed, setpoint),
      //  might have to swith one is negative
      //  left(counterclockwise) should be negative and right(clockwise) should be positive
      -Math.copySign(RobotContainer.driveConstants.kAutoDriveSpeed, setpoint));
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drive.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
      return Math.abs(RobotContainer.navX.getAngle()) > (zeroPos + setpoint);
  }
}
