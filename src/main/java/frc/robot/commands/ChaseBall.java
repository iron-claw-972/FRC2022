package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private PIDController followPID = new PIDController(limelightConstants.kFollowP, limelightConstants.kFollowI, limelightConstants.kFollowD);

  public ChaseBall(Limelight limelight, Drivetrain drivetrain, boolean isRedBall) {
    m_limelight = limelight;
    m_drive = drivetrain;
    addRequirements(limelight, drivetrain);

    m_isRedBall = isRedBall;
    chasePID.setTolerance(limelightConstants.kChasePIDTolerance);
    chasePID.reset();
    // chasePID.setSetpoint(0);

    // followPID.setTolerance(limelightConstants.kFollowPIDTolerance);
    followPID.reset();
    //followPID.setSetpoint(6);
  }

  @Override
  public void initialize() {
    chasePID.reset();
    followPID.reset();
  }

  @Override
  public void execute() {
    if (m_limelight.hasValidTarget()) {
      double distance = -Units.metersToInches(m_limelight.getBallDistance(2, true));
      System.out.println("Distance: " + distance);
      double pid = followPID.calculate(distance, 6);
      System.out.println("PID: " + pid);
      m_drive.arcadeDrive(MathUtil.clamp(pid, -0.5, 0.5), chasePID.calculate(m_limelight.getBallHorizontalAngularOffset(true), 0));
    }
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