package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotConstants.limelight.MarinusLimelightConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class ChaseBall extends CommandBase {
  public static MarinusLimelightConstants limelightConstants = new MarinusLimelightConstants();

  private final Limelight m_limelight;
  private final Drivetrain m_drive;
  
  private final boolean m_isRedBall;

  private PIDController chasePID = new PIDController(limelightConstants.kChaseP, limelightConstants.kChaseI, limelightConstants.kChaseD);
  private PIDController followPID = new PIDController(limelightConstants.kFollowP, limelightConstants.kFollowI, limelightConstants.kFollowD);

  public ChaseBall(Limelight limelight, Drivetrain drivetrain, boolean isRedBall) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    m_isRedBall = isRedBall;

    chasePID.setTolerance(limelightConstants.kChasePIDTolerance);
    SmartDashboard.putData("Turn chase PID", chasePID);

    followPID.setTolerance(limelightConstants.kFollowPIDTolerance);
    SmartDashboard.putData("Throttle chase PID", followPID);
  }

  @Override
  public void initialize() {
    m_limelight.setBallPipeline(m_isRedBall);
    chasePID.reset();
    followPID.reset();
  }

  @Override
  public void execute() {
    double distance = Units.metersToInches(m_limelight.getBallDistance(2, m_isRedBall));

    m_drive.arcadeDrive(
      MathUtil.clamp(followPID.calculate(distance, 0), -0.5, 0.5),
      MathUtil.clamp(chasePID.calculate(m_limelight.getBallHorizontalAngularOffset(m_isRedBall), 0), -0.5, 0.5)
    );
  }

  @Override
  public boolean isFinished() {
    return ShooterMethods.isBallContained();
  }

  @Override
  public void end(boolean interrupted) {
    m_limelight.setCameraMode(true);
    m_drive.arcadeDrive(0, 0);
  }
}