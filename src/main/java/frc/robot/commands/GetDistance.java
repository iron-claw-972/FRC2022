package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.limelight.TraversoLimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class GetDistance extends CommandBase {
  private final Limelight m_limelight;

  public static TraversoLimelightConstants constants = new TraversoLimelightConstants();

  public static boolean isFinished = false;
  public static double optimalVelocity = Double.NaN;
  public static double optimalAngle = Double.NaN;

  private boolean isFront = true;

  public GetDistance(Limelight limelight) {
    m_limelight = limelight;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    isFinished = false;
    optimalVelocity = Double.NaN;
    optimalAngle = Double.NaN;
  }

  @Override
  public void execute() {
    // Get distance from limelight
    double stipeAngle = ShooterMethods.getArmAngle();
    double limelightAngle = stipeAngle + constants.kStipeToLimelightAngularOffset;
    double shooterAngle = stipeAngle + RobotContainer.cargoConstants.kStipeToShootingAngularOffset;
    if (limelightAngle > 90) {
      isFront = false;
      limelightAngle = 180 - limelightAngle;
    } else {
      isFront = true;
    }
    double distance = m_limelight.getHubDistance(limelightAngle);
    if (!Double.isNaN(distance)) {
      double limelightAngleRad = Units.degreesToRadians(limelightAngle);
      double shooterAngleRad = Units.degreesToRadians(shooterAngle);
      double shooterDistance = distance + (constants.kHubWidth / 2) - (constants.kPivotToLimelightDistance * Math.cos(limelightAngleRad)) + (RobotContainer.cargoConstants.kPivotToShootingExitPoint * Math.cos(shooterAngleRad));
      double targetHeightOffset = constants.kHubHeight - constants.kPivotHeight - (RobotContainer.cargoConstants.kPivotToShootingExitPoint * Math.sin(shooterAngleRad));

      optimalAngle = ShooterMethods.getOptimalShootingAngle(RobotContainer.cargoConstants.kSAngle, shooterDistance, targetHeightOffset);
      if (!isFront) {
        optimalAngle = 180 - optimalAngle;
      }
      double optimalStipeAngle = optimalAngle - RobotContainer.cargoConstants.kStipeToShootingAngularOffset;
      optimalVelocity = Units.metersToFeet(ShooterMethods.getOptimalShooterSpeed(optimalStipeAngle, targetHeightOffset, distance));
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return !(Double.isNaN(optimalAngle) || Double.isNaN(optimalVelocity)) || isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }
}
  