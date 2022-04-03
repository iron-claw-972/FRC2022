/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives a certain distance
 */
public class DriveRotation extends CommandBase {
  double setpoint, zeroPos;

  public static boolean isFinished = false;

  public Drivetrain m_drive;

  public DriveRotation(double setpoint) {
    this(setpoint, Robot.drive);
  }

  public DriveRotation(double setpoint, Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    zeroPos = m_drive.getLeftPosition();
    // m_drive.setBrakeMode();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    m_drive.tankFeedForwardDrive(
      -Math.copySign(Constants.auto.kDriveSpeed, setpoint),
       Math.copySign(Constants.auto.kDriveSpeed, setpoint));
  }

  // @Override
  // public boolean isFinished() {
  //   return Math.abs(m_drive.getLeftPosition()) > (zeroPos + setpoint);
  // }

  @Override
  public boolean isFinished() {
    if (setpoint > 0) {
      return -m_drive.getLeftPosition() <= zeroPos - setpoint;
    } else {
      return -m_drive.getLeftPosition() >= zeroPos - setpoint;
    }
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = true;
    m_drive.tankDriveVolts(0, 0);
  }
}
