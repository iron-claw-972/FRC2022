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
public class DriveDistance extends CommandBase {
  double setpoint, zeroPos;

  public static boolean isFinished = false;

  public DriveDistance(double setpoint_) {
    addRequirements(RobotContainer.m_drive);
    setpoint = setpoint_;
  }

  @Override
  public void initialize() {
    zeroPos = RobotContainer.m_drive.getLeftPosition();
    RobotContainer.m_drive.setBrakeMode();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_drive.tankDrive(-Math.copySign(RobotContainer.driveConstants.kAutoDriveSpeed, setpoint), -Math.copySign(RobotContainer.driveConstants.kAutoDriveSpeed, setpoint));
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = true;
    RobotContainer.m_drive.tankDrive(0, 0);
    RobotContainer.m_drive.setCoastMode();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.m_drive.getLeftPosition()  - zeroPos) >= Math.abs(setpoint);
  }

}
