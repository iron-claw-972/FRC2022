package frc.robot.commands.cargoCommands;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
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
        // Start spinning up
        new InstantCommand(() -> ShooterMethods.setWheelRPM(2000)),

        new InstantCommand(() -> ShooterMethods.enableAll()),

        parallel(
          // Set hub pipeline early to account for network latency
          new InstantCommand(() -> RobotContainer.m_limelight.setUpperHubPipeline()).withTimeout(0.5),

          // Spin up belt and wheels in advance
          sequence(
            // Don't spin shooting wheels until ball is confirmed to be in shooter
            new InstantCommand(() -> ShooterMethods.setBeltPower(RobotContainer.beltConstants.kIntakeSpeed)),
            new WaitUntilCommand(() -> ShooterMethods.isBallContained()).withTimeout(0.5)
          ),

          sequence(
            // Spin up wheels to optimal velocity when the limelight gets the optimal angle
            new ConditionalCommand(
              sequence(
                new WaitUntilCommand(() -> GetDistance.isFinished),
                new InstantCommand(() -> ShooterMethods.setWheelSpeed(() -> GetDistance.optimalVelocity, isFront))
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
              new PositionArm((isFront ? RobotContainer.cargoConstants.kFrontLimelightScanPos : RobotContainer.cargoConstants.kBackLimelightScanPos)).withTimeout(2),
              new DoNothing(), 
              () -> doesCalculateSpeed || doesAlign
            ),

            // Align using limelight
            // new ConditionalCommand(
            //   new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive).withTimeout(1),
            //   new DoNothing(),
            //   () -> doesAlign
            // ),
            //  Calculate distance and determines optimal shooting angle and velocity and
            // set to actual shooting angle
            new ConditionalCommand(
              sequence(
                new GetDistance(RobotContainer.m_limelight, RobotContainer.m_cargoRotator).withTimeout(1.5),
                new PositionArmOptimal().withTimeout(1)
              ),
              new PositionArm(outtakeArmPosition),
              () -> doesCalculateSpeed
            )
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
      GetDistance.isFinished = false; // Reset finished condition
    }
}
