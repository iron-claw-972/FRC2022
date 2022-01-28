/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.Driver;
import frc.robot.controls.Functions;
import frc.robot.subsystems.Drivetrain;

public class RunDrive extends CommandBase {
  private final Drivetrain m_drive;

  public RunDrive(Drivetrain subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //TODO: acceleration smoothing
    // System.out.println("running command");
    m_drive.runDrive(Driver.getRawThrottleValue(), Driver.getRawTurnValue());
    SmartDashboard.putBoolean("arcade drive", m_drive.isDrive("arcade"));
    SmartDashboard.putBoolean("prop drive", m_drive.isDrive("prop"));
    SmartDashboard.putBoolean("shift drive", m_drive.isDrive("shift"));
  }
}
