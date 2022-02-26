package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignToUpperHub extends CommandBase {
  private final Limelight m_limelight;
  private final Drivetrain m_drive;

  private PIDController alignPID = new PIDController(0.03, 0, 0);

  public AlignToUpperHub(Limelight limelight, Drivetrain drivetrain) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);
  }

  @Override
  public void execute() {
    m_drive.runDrive(0, alignPID.calculate(m_limelight.getHubHorizontalAngularOffset(), 0));
  }
}