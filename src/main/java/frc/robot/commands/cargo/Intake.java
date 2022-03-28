package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.constants.Constants;
import frc.robot.util.ShooterMethods;

public class Intake extends SequentialCommandGroup {
  public Intake(
      double postIntakeArmPosition,
      boolean doesChaseBall
  ) {
    this(postIntakeArmPosition, doesChaseBall, true, true);
  }

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
    addRequirements(Robot.m_shooter, Robot.m_arm, Robot.m_belt,
        Robot.m_limelight);
    addCommands(
        // Set pipeline early to account for network latency
        new InstantCommand(() -> Robot.m_limelight.setBallPipeline(isRedBall)),

        new InstantCommand(() -> ShooterMethods.enableAll()),

        // new ConditionalCommand(
        //   new WaitUntilCommand(() -> !ShooterMethods.isBallContained()),
        //   new DoNothing(), 
        //   () -> doesCheckBall
        // ),

        // Spin up wheel, belt, set angle, and start ball chase simultaneously
        parallel(
          new InstantCommand(() -> ShooterMethods.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setBeltPower(Constants.belt.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setAngle(Constants.arm.kIntakePos)),
          new ConditionalCommand(
            new ChaseBall(Robot.m_limelight, Robot.m_drive, isRedBall),
            new TeleopDrive(Robot.m_drive),
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
    // Robot.m_limelight.setDriverPipeline();
  }
}
