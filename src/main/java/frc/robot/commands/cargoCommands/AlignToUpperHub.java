package frc.robot.commands.cargoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotConstants.limelight.MarinusLimelightConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignToUpperHub extends CommandBase {
  public static MarinusLimelightConstants limelightConstants = new MarinusLimelightConstants();

  private final Limelight m_limelight;
  private final Drivetrain m_drive;

  public static boolean isFinished = false;
  public static double offset;

  public static PIDController alignPID = new PIDController(limelightConstants.kAlignP, limelightConstants.kAlignI, limelightConstants.kAlignD);

  public AlignToUpperHub(Limelight limelight, Drivetrain drivetrain) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    alignPID.setTolerance(limelightConstants.kAlignPIDTolerance);
  }

  @Override
  public void initialize() {
    isFinished = false;
    m_limelight.setUpperHubPipeline();
    alignPID.reset();
  }

  @Override
  public void execute() {
    offset = m_limelight.getHubHorizontalAngularOffset();
    m_drive.arcadeDrive(0, alignPID.calculate(offset, 0));
  }

  @Override
  public boolean isFinished() {
    return alignPID.atSetpoint() && m_limelight.hasValidTarget();
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = true;
    m_drive.arcadeDrive(0, 0);
  }
}