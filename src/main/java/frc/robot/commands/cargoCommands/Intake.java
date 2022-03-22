package frc.robot.commands.cargoCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.driveCommands.TeleopDrive;
import frc.robot.util.ShooterMethods;

public class Intake extends SequentialCommandGroup {
  public Intake(
      double postIntakeArmPosition,
      boolean doesChaseBall,
      boolean isRedBall
  ) {
    this(postIntakeArmPosition, doesChaseBall, isRedBall, true);
  }

  public Intake(
      double postIntakeArmPosition,
      boolean doesChaseBall,
      boolean isRedBall,
      boolean doesCheckBall
  ) {
    addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt,
        RobotContainer.m_limelight);
    addCommands(
        new InstantCommand(() -> ShooterMethods.enableAll()),
        // Set pipeline early to account for network latency
        new InstantCommand(() -> RobotContainer.m_limelight.setBallPipeline(isRedBall)),

        new ConditionalCommand(
          new WaitUntilCommand(() -> !ShooterMethods.isBallContained()),
          new DoNothing(), 
          () -> doesCheckBall
        ),

        // Spin up wheel, belt, set angle, and start ball chase simultaneously
        parallel(
          new InstantCommand(() -> ShooterMethods.setWheelRPM(RobotContainer.wheelConstants.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setBeltPower(RobotContainer.beltConstants.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setAngle(RobotContainer.cargoConstants.kIntakePos)),
          new ConditionalCommand(
            new ChaseBall(RobotContainer.m_limelight, RobotContainer.m_drive, isRedBall),
            new TeleopDrive(RobotContainer.m_drive),
            () -> doesChaseBall
          )
        ),

        // new DoNothing()
        new ConditionalCommand(
          sequence(
            // Bring arm back up and stop intake when we have the ball
            new WaitUntilCommand(() -> ShooterMethods.isBallContained()),
            new InstantCommand(() -> ShooterMethods.disableShiitake()),
            new InstantCommand(() -> ShooterMethods.setAngle(postIntakeArmPosition)),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1)
          ),
          new DoNothing(), 
          () -> doesCheckBall
        )
    );
  }

  @Override
  public void end(boolean interrupted) {
    ShooterMethods.disableShiitake();
    // RobotContainer.m_limelight.setDriverPipeline();
  }
}
