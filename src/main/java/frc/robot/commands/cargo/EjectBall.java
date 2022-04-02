package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.util.CargoUtil;

public class EjectBall extends SequentialCommandGroup {
  public EjectBall() {
    this(Robot.belt, Robot.shooter, Robot.ballDetection);
  }

  public EjectBall(Belt belt, Shooter shooter, BallDetection ballDetection) {
    addRequirements(belt, shooter, ballDetection);
    addCommands(
      new InstantCommand(() -> CargoUtil.setBeltPower(0.8)),
      new InstantCommand(() -> CargoUtil.setWheelRPM(-2000)),
      new InstantCommand(() -> CargoUtil.enableBelt()),
      new InstantCommand(() -> CargoUtil.enableWheel()),
      new WaitUntilCommand(() -> !CargoUtil.isBallContained()),
      new WaitCommand(0.5)
    );
  }

  @Override
  public void end(boolean interrupted) {
    CargoUtil.disableShiitake();
    // Robot.m_limelight.setDriverPipeline();
  }
}
