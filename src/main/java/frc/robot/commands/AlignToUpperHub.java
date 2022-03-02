package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotConstants.limelight.TraversoLimelightConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class AlignToUpperHub extends CommandBase {
  public static TraversoLimelightConstants limelightConstants = new TraversoLimelightConstants();

  private final Limelight m_limelight;
  private final Drivetrain m_drive;

  private PIDController alignPID = new PIDController(limelightConstants.kAlignP, limelightConstants.kAlignI, limelightConstants.kAlignD);

  public AlignToUpperHub(Limelight limelight, Drivetrain drivetrain) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    alignPID.setTolerance(limelightConstants.kAlignPIDTolerance);
    alignPID.reset();
    alignPID.setSetpoint(0);
    SmartDashboard.putData("Alignment PID", alignPID);
  }

  @Override
  public void execute() {
  // System.out.println("Angle: " + m_limelight.getHubHorizontalAngularOffset());
    m_drive.runDrive(0, alignPID.calculate(m_limelight.getHubHorizontalAngularOffset()));
  }

  @Override
  public boolean isFinished() {
    return alignPID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_limelight.setCameraMode(true);
    System.out.println("Alignment finished");
  }
}