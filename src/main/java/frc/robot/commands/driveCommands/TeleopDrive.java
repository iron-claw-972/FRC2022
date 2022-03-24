/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
  private final Drivetrain m_drive;

  public TeleopDrive(Drivetrain subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = Driver.getTurnValue();
    // double turnBoost = SmartDashboard.getNumber("Turn Boost", 0.2);
    // if (turn > 0 && turn < turnBoost) {
    //   turn = turnBoost;
    // }
    // if (turn < 0 && turn > -turnBoost) {
    //   turn = -turnBoost;
    // }
    double throttle = Driver.getThrottleValue();
    /*double throttleBoost = SmartDashboard.getNumber("Throttle Boost", 0.05);
    if (throttle > 0 && throttle < throttleBoost) {
      throttle = throttleBoost;
    }
    if (throttle < 0 && throttle > -throttleBoost) {
      throttle = -throttleBoost;
    }*/
    m_drive.runDrive(throttle, -0.85 * turn);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }
  
}
