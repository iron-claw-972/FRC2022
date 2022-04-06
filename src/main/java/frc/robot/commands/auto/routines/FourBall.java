
package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.PathweaverIntake;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.util.CargoUtil;

public class FourBall extends SequentialCommandGroup {
  public FourBall(Alliance color) {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll, Robot.ballDetection, color);
  }

  public FourBall(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight limelight, BallDetection ballDetection, Alliance color) {
    addRequirements(drive, belt, arm, shooter, limelight, ballDetection);
    addCommands(
      new ThreeBall(color),
      
      new PathweaverIntake("4ballthree", false),

      new InstantCommand(() -> CargoUtil.setBeltSpeed(Constants.belt.kIntakeSpeed)),
      new InstantCommand(() -> CargoUtil.setWheelSpeed(() -> 32, true)),

      new PathweaverCommand("4ballfour", false, true),
      
      new ShootAuto(false, true, 0, () -> true, 100, 32),

      new PositionArm(154)
    );
  }
}
