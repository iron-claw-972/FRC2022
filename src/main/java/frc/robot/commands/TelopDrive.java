/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class TelopDrive extends CommandBase {
  private final Drivetrain m_drive;

  public TelopDrive(Drivetrain subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.runDrive(Driver.getRawThrottleValue(), Driver.getRawTurnValue());
  }
}
