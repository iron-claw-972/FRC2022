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

/**
 * Drives a certain distance
 */
public class DriveRotation extends CommandBase {
  double setpoint, zeroPos;

  public static boolean isFinished = false;

  public DriveRotation(double setpoint_) {
    addRequirements(Robot.m_drive);
    setpoint = setpoint_;
  }

  @Override
  public void initialize() {
    zeroPos = Robot.m_drive.getLeftPosition();
    // Robot.m_drive.setBrakeMode();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    Robot.m_drive.tankDrive(
      -Math.copySign(Constants.auto.kDriveSpeed, setpoint),
       Math.copySign(Constants.auto.kDriveSpeed, setpoint));
  }

  // @Override
  // public boolean isFinished() {
  //   return Math.abs(Robot.m_drive.getLeftPosition()) > (zeroPos + setpoint);
  // }

  @Override
  public boolean isFinished() {
    if (setpoint > 0) {
      return Robot.m_drive.getLeftPosition() <= zeroPos - setpoint;
    } else {
      return Robot.m_drive.getLeftPosition() >= zeroPos - setpoint;
    }
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = true;
    Robot.m_drive.tankDrive(0, 0);
  }
}
