/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.ShuffleboardManager;

/**
 * Drives a certain distance
 */
public class DriveDistance extends CommandBase {
  double setpoint, zeroPos;

  public DriveDistance(double setpoint_) {
    addRequirements(RobotContainer.m_drive);
    setpoint = setpoint_;
  }

  @Override
  public void initialize() {
    zeroPos = RobotContainer.m_drive.getLeftPosition();
    RobotContainer.m_drive.setBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_drive.tankDrive(-0.4, -0.4);//IDK why this is 0.5, then .6
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drive.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
      // return false;
      return RobotContainer.m_drive.getLeftPosition() < zeroPos - setpoint;
  }
  
}
