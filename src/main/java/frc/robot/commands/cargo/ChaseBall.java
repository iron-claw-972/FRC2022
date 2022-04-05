package frc.robot.commands.cargo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.CargoUtil;

public class ChaseBall extends CommandBase {
  private final Limelight m_limelight;
  private final Drivetrain m_drive;
  
  private final boolean m_isRedBall;
  public static double offset;

  private boolean m_driverControlled;

  public static PIDController turnPID = new PIDController(Constants.ll.kTurnP, Constants.ll.kTurnI, Constants.ll.kTurnD);
  // private PIDController throttlePID = new PIDController(Constants.ll.kThrottleP, Constants.ll.kThrottleI, Constants.ll.kThrottleD);

  public ChaseBall(boolean isRedBall) {
    this(isRedBall, true);
  }

  public ChaseBall(boolean isRedBall, boolean driverControlled) {
    this(Robot.ll, Robot.drive, isRedBall, driverControlled);
  }

  public ChaseBall(Limelight limelight, Drivetrain drivetrain, boolean isRedBall, boolean driverControlled) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    m_driverControlled = driverControlled;
    m_isRedBall = isRedBall;

    turnPID.setTolerance(Constants.ll.kTurnPIDTolerance);
    // throttlePID.setTolerance(Constants.ll.kThrottlePIDTolerance);
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
    double turn = MathUtil.clamp(
      turnPID.calculate(offset, 0),
      -Constants.ll.kMaxTurnPower,
      Constants.ll.kMaxTurnPower
    );
    m_drive.feedForwardDrive(
      // MathUtil.clamp(throttlePID.calculate(distance, 0), -0.5, 0.5),
      (m_driverControlled ? Driver.getThrottleValue() * Constants.drive.kMaxSpeedMetersPerSecond : Constants.ll.kAutoThrottlePow),
      (Double.isNaN(turn) ? Driver.getTurnValue() : turn) * Constants.drive.kMaxAngularSpeedRadiansPerSecond
    );
  }

  @Override
  public boolean isFinished() {
    return CargoUtil.isBallContained();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
    m_limelight.setDriverPipeline();
  }
}