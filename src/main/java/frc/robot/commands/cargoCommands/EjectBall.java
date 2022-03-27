package frc.robot.commands.cargoCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.driveCommands.TeleopDrive;
import frc.robot.util.ShooterMethods;

public class EjectBall extends SequentialCommandGroup {
  public EjectBall() {
    addRequirements(RobotContainer.m_cargoBelt, RobotContainer.m_cargoShooter);
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
    // RobotContainer.m_limelight.setDriverPipeline();
  }
}
