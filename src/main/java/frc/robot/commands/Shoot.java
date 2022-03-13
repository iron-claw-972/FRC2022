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
        this(doesCalculateSpeed, doesAlign, isFront, 0, () -> true);
    }

    public Shoot(boolean doesCalculateSpeed, boolean doesAlign, boolean isFront, double waitTime, BooleanSupplier waitCondition) {
        this(doesCalculateSpeed, doesAlign, isFront, waitTime, waitCondition, Double.NaN, Double.NaN);
    }

    public Shoot(
          boolean doesCalculateSpeed,
          boolean doesAlign,
          boolean isFront,
          double waitTime,
          BooleanSupplier waitCondition,
          double outtakeArmPosition,
          double shooterWheelOuttakeSpeed
    ) {
      addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt);
      addCommands(
        new InstantCommand(() -> ShooterMethods.enableAll()),
        new InstantCommand(() -> RobotContainer.m_limelight.setCameraMode(false)),
        parallel(
          // Wait a certain time before shooting
          new WaitCommand(waitTime),

          // Spin up belt and wheels
          sequence(
            new InstantCommand(() -> ShooterMethods.setBeltPower(RobotContainer.beltConstants.kIntakeSpeed)),
            new WaitUntilCommand(() -> ShooterMethods.isBallContained()).withTimeout(0.5),
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
            new ConditionalCommand(
              new WaitUntilCommand(waitCondition),
              new DoNothing(),
              () -> doesAlign
            ),
            parallel(
              new ConditionalCommand(
                new DoNothing(),
                new WaitUntilCommand(waitCondition),
                () -> doesAlign
              ),
              sequence(
                new ConditionalCommand(
                  sequence(
                    new ConditionalCommand(
                      new InstantCommand(() -> ShooterMethods.setAngle((isFront ? RobotContainer.cargoConstants.kFrontLimelightScanPos : RobotContainer.cargoConstants.kBackLimelightScanPos))),
                      new InstantCommand(() -> ShooterMethods.setAngle(outtakeArmPosition)),
                      () -> doesCalculateSpeed || doesAlign
                    ),
                    new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1)
                  ), 
                  new DoNothing(), 
                  () -> doesCalculateSpeed || doesAlign
                ),
                new ConditionalCommand(
                  new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive).withTimeout(3),
                  new DoNothing(),
                  () -> doesAlign
                ),
                new ConditionalCommand(
                  new GetDistance(RobotContainer.m_limelight),
                  new DoNothing(),
                  () -> doesCalculateSpeed
                ),
                new InstantCommand(() -> ShooterMethods.setAngle(() -> GetDistance.optimalAngle)),
                new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1)
              )
            )
          )
        ),
        new InstantCommand(() -> ShooterMethods.setBeltPower(RobotContainer.beltConstants.kOuttakeSpeed)),
        new WaitCommand(0.6)
      );
    }

    @Override
    public void end(boolean interrupted) {
        ShooterMethods.disableShiitake();
    }
}
