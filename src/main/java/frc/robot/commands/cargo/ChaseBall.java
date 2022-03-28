package frc.robot.commands.cargo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class ChaseBall extends CommandBase {
  private final Limelight mLimelight;
  private final Drivetrain m_drive;
  
  private final boolean m_isRedBall;
  public static double offset;

  private boolean driverControl;

  public static PIDController turnPID = new PIDController(Constants.ll.kTurnP, Constants.ll.kTurnI, Constants.ll.kTurnD);
  // private PIDController throttlePID = new PIDController(Constants.ll.kThrottleP, Constants.ll.kThrottleI, Constants.ll.kThrottleD);

  public ChaseBall(Limelight limelight, Drivetrain drivetrain, boolean isRedBall, boolean driverControlled) {
    this(limelight, drivetrain, isRedBall);
    driverControl = driverControlled;
  }

  public ChaseBall(Limelight limelight, Drivetrain drivetrain, boolean isRedBall) {
    mLimelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    m_isRedBall = isRedBall;

    turnPID.setTolerance(Constants.ll.kTurnPIDTolerance);

    driverControl = true;
    // throttlePID.setTolerance(Constants.ll.kThrottlePIDTolerance);
    // SmartDashboard.putData("Throttle chase PID", followPID);
  }

  @Override
  public void initialize() {
    mLimelight.setBallPipeline(m_isRedBall);
    turnPID.reset();
    // throttlePID.reset();
  }

  @Override
  public void execute() {
    // double distance = Units.metersToInches(mLimelight.getBallDistance(2, m_isRedBall));

    offset = mLimelight.getBallHorizontalAngularOffset(m_isRedBall);
    double turn = MathUtil.clamp(
      turnPID.calculate(offset, 0),
      -Constants.ll.kMaxTurnPower,
      Constants.ll.kMaxTurnPower
    );
    m_drive.arcadeDrive(
      // MathUtil.clamp(throttlePID.calculate(distance, 0), -0.5, 0.5),
      (driverControl ? Driver.getThrottleValue() : Constants.ll.kAutoThrottlePow),
      (Double.isNaN(turn) ? -Driver.getTurnValue() : turn)
    );
  }

  @Override
  public boolean isFinished() {
    return ShooterMethods.isBallContained();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    Robot.m_limelight.setDriverPipeline();
  }
}