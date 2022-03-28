package frc.robot.commands.cargo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignToUpperHub extends CommandBase {
  private final Limelight m_limelight;
  private final Drivetrain m_drive;

  public static boolean isFinished = false;
  public static double offset;

  public static PIDController alignPID = new PIDController(Constants.ll.kAlignP, Constants.ll.kAlignI, Constants.ll.kAlignD);

  public AlignToUpperHub(Limelight limelight, Drivetrain drivetrain) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    alignPID.setTolerance(Constants.ll.kAlignPIDTolerance);
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