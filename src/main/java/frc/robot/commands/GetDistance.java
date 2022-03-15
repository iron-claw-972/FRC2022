package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.limelight.TraversoLimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class GetDistance extends CommandBase {
  private final Limelight m_limelight;

  public static TraversoLimelightConstants constants = new TraversoLimelightConstants();

  public static boolean isFinished = false;
  public static double optimalVelocity = Double.NaN;
  public static double optimalShooterAngle = Double.NaN;
  public static double distance = Double.NaN;

  private boolean isFront = true;

  public GetDistance(Limelight limelight) {
    m_limelight = limelight;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    isFinished = false;
    m_limelight.setUpperHubPipeline();
    optimalVelocity = Double.NaN;
    optimalShooterAngle = Double.NaN;
    distance = Double.NaN;
  }

  @Override
  public void execute() {
    // Get distance from limelight
    double stipeAngle = ShooterMethods.getArmAngle();
    double limelightAngle = stipeAngle + constants.kStipeToLimelightAngularOffset;
    double shooterAngle = stipeAngle + RobotContainer.cargoConstants.kStipeToShootingAngularOffset;
    isFront = limelightAngle < 90;
    distance = Units.metersToInches(m_limelight.getHubDistance(stipeAngle));

    if (!Double.isNaN(distance)) {
      // Get arguments of optimal shooting equations
      double limelightAngleRad = Units.degreesToRadians(limelightAngle);
      double shooterAngleRad = Units.degreesToRadians(shooterAngle);
      double shooterDistance = distance + (constants.kHubWidth / 2) - (constants.kPivotToLimelightDistance * Math.cos(limelightAngleRad)) + (RobotContainer.cargoConstants.kPivotToShootingExitPoint * Math.cos(shooterAngleRad));
      double targetHeightOffset = constants.kHubHeight - constants.kPivotHeight - (RobotContainer.cargoConstants.kPivotToShootingExitPoint * Math.sin(shooterAngleRad));

      // Find optimal shooting angle
      optimalShooterAngle = ShooterMethods.getOptimalShootingAngle(RobotContainer.cargoConstants.kSAngle, shooterDistance, targetHeightOffset);
      if (!isFront) {
        optimalShooterAngle = 180 - optimalShooterAngle;
      }
      double optimalStipeAngle = optimalShooterAngle - RobotContainer.cargoConstants.kStipeToShootingAngularOffset;

      // Find optimal shooting velocity
      optimalVelocity = Units.metersToFeet(ShooterMethods.getOptimalShooterSpeed(optimalStipeAngle, targetHeightOffset, distance));
      optimalVelocity *= -1;
      optimalVelocity *= RobotContainer.wheelConstants.kShotEfficiency;
    } else {
      optimalVelocity = Double.NaN;
      optimalShooterAngle = Double.NaN;
      distance = Double.NaN;
    }
  }

  @Override
  public boolean isFinished() {
    return !(Double.isNaN(optimalShooterAngle) || Double.isNaN(optimalVelocity));
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }
}
  