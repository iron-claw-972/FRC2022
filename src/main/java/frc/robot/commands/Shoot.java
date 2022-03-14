package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ShooterMethods;

public class Shoot extends SequentialCommandGroup {
    public Shoot(boolean doesCalculateSpeed, boolean doesAlign, boolean isFront) {
        this(doesCalculateSpeed, doesAlign, isFront, Double.NaN, Double.NaN);
    }

    public Shoot(
          boolean doesCalculateSpeed,
          boolean doesAlign,
          boolean isFront,
          double outtakeArmPosition,
          double shooterWheelOuttakeSpeed
    ) {
      addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt, RobotContainer.m_limelight);
      addCommands(
        new InstantCommand(() -> ShooterMethods.enableAll()),
        // Set hub pipeline early to account for network latency
        new InstantCommand(() -> RobotContainer.m_limelight.setUpperHubPipeline()),

        parallel(
          // Spin up belt and wheels
          sequence(
            // Don't spin shooting wheels until ball is confirmed to be in shooter
            new InstantCommand(() -> ShooterMethods.setBeltPower(RobotContainer.beltConstants.kIntakeSpeed)),
            new WaitUntilCommand(() -> ShooterMethods.isBallContained()).withTimeout(0.5),

            // Spin up wheels to optimal velocity when the limelight gets the optimal angle
            new ConditionalCommand(
              sequence(
                new WaitUntilCommand(() -> GetDistance.isFinished),
                new InstantCommand(() -> ShooterMethods.setWheelSpeed(() -> GetDistance.optimalVelocity))
              ),
              new InstantCommand(() -> ShooterMethods.setWheelRPM(shooterWheelOuttakeSpeed)),
              () -> doesCalculateSpeed
            ),

            new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint())
          ),

          // Limelight stuff
          sequence(
            // Get arm to limelight angle
            new ConditionalCommand(
              sequence(
                new InstantCommand(() -> ShooterMethods.setAngle((isFront ? RobotContainer.cargoConstants.kFrontLimelightScanPos : RobotContainer.cargoConstants.kBackLimelightScanPos))),
                new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1)
              ),
              new DoNothing(), 
              () -> doesCalculateSpeed || doesAlign
            ),

            // Align using limelight
            new ConditionalCommand(
              new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive).withTimeout(3),
              new DoNothing(),
              () -> doesAlign
            ),
            //  Collect vision data
            new ConditionalCommand(
              new GetDistance(RobotContainer.m_limelight),
              new DoNothing(),
              () -> doesCalculateSpeed
            ),

            // Set to actual shooting angle
            new ConditionalCommand(
              new InstantCommand(() -> ShooterMethods.setAngle(() -> GetDistance.optimalAngle)),
              new InstantCommand(() -> ShooterMethods.setAngle(outtakeArmPosition)),
              () -> doesAlign || doesCalculateSpeed
            ),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1)
          )
        ),

        // Spin belts to outtake ball
        new InstantCommand(() -> ShooterMethods.setBeltPower(RobotContainer.beltConstants.kOuttakeSpeed)),
        new WaitCommand(0.5)
      );
    }

    @Override
    public void end(boolean interrupted) {
        ShooterMethods.disableShiitake();
    }
}
