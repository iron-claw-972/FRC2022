package frc.robot.commands.cargoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.Driver;
import frc.robot.robotConstants.limelight.MarinusLimelightConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class ChaseBall extends CommandBase {
  public static MarinusLimelightConstants limelightConstants = new MarinusLimelightConstants();

  private final Limelight m_limelight;
  private final Drivetrain m_drive;
  
  private final boolean m_isRedBall;
  public static double offset;

  public static PIDController turnPID = new PIDController(limelightConstants.kTurnP, limelightConstants.kTurnI, limelightConstants.kTurnD);
  // private PIDController throttlePID = new PIDController(limelightConstants.kThrottleP, limelightConstants.kThrottleI, limelightConstants.kThrottleD);

  public ChaseBall(Limelight limelight, Drivetrain drivetrain, boolean isRedBall) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    m_isRedBall = isRedBall;

    turnPID.setTolerance(limelightConstants.kTurnPIDTolerance);

    // throttlePID.setTolerance(limelightConstants.kThrottlePIDTolerance);
    // SmartDashboard.putData("Throttle chase PID", followPID);
  }

  @Override
  public void initialize() {
    m_limelight.setBallPipeline(m_isRedBall);
    turnPID.reset();
    // throttlePID.reset();
  }

  @Override
  public void execute() {
    // double distance = Units.metersToInches(m_limelight.getBallDistance(2, m_isRedBall));

    offset = m_limelight.getBallHorizontalAngularOffset(m_isRedBall);
    m_drive.arcadeDrive(
      // MathUtil.clamp(throttlePID.calculate(distance, 0), -0.5, 0.5),
      Driver.getThrottleValue(),
      MathUtil.clamp(
        turnPID.calculate(offset, 0),
        -RobotContainer.limelightConstants.kMaxTurnPower,
        RobotContainer.limelightConstants.kMaxTurnPower
      )
    );
  }

  @Override
  public boolean isFinished() {
    return ShooterMethods.isBallContained();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }
}