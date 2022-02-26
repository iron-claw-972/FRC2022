package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotConstants.limelight.TraversoLimelightConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class ChaseBall extends CommandBase {
  public static TraversoLimelightConstants limelightConstants = new TraversoLimelightConstants();

  private final Limelight m_limelight;
  private final Drivetrain m_drive;
  
  private final boolean m_isRedBall;

  private PIDController chasePID = new PIDController(limelightConstants.kChaseP, limelightConstants.kChaseI, limelightConstants.kChaseD);

  public ChaseBall(Limelight limelight, Drivetrain drivetrain, boolean isRedBall) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    m_isRedBall = isRedBall;
    chasePID.setTolerance(limelightConstants.kChasePIDTolerance);
    chasePID.reset();
    chasePID.setSetpoint(0);
  }

  @Override
  public void execute() {
    m_drive.runDrive(0, chasePID.calculate(m_limelight.getBallHorizontalAngularOffset(m_isRedBall)));
  }

  @Override
  public boolean isFinished() {
    return ShooterMethods.isBallContained();
  }

  @Override
  public void end(boolean interrupted) {
    m_limelight.setCameraMode(true);
  }
}