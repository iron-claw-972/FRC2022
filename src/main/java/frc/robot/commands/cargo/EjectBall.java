package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.util.ShooterMethods;

public class EjectBall extends SequentialCommandGroup {
  public EjectBall() {
    addRequirements(Robot.m_belt, Robot.m_shooter);
    addCommands(
      new InstantCommand(() -> ShooterMethods.setBeltPower(0.8)),
      new InstantCommand(() -> ShooterMethods.setWheelRPM(-1500)),
      new InstantCommand(() -> ShooterMethods.enableBelt()),
      new InstantCommand(() -> ShooterMethods.enableWheel()),
      new WaitUntilCommand(() -> !ShooterMethods.isBallContained()),
      new WaitCommand(0.5)
    );
  }

  @Override
  public void end(boolean interrupted) {
    ShooterMethods.disableShiitake();
    // Robot.m_limelight.setDriverPipeline();
  }
}
