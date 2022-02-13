package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Shuffleboard;

public class ShuffleboardUpdate extends CommandBase {
  private final Shuffleboard m_shuffleboard;

  public ShuffleboardUpdate(Shuffleboard subsystem) {
    m_shuffleboard = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {

    //drive mode
    m_shuffleboard.m_driveModeChooser.setDefaultOption("Arcade Drive", "Arcade Drive");
    m_shuffleboard.m_driveModeChooser.addOption("Shift Drive", "Shift Drive");
    m_shuffleboard.m_driveModeChooser.addOption("Proportional Drive", "Proportional Drive");
    m_shuffleboard.m_driveModeChooser.addOption("Tank Drive", "Tank Drive");
    SmartDashboard.putData("Drive Mode", m_shuffleboard.m_driveModeChooser);
    //SmartDashboard.putBoolean("Has Ball", RobotContainer.getBallDetection().containsBall());
  }
}
  