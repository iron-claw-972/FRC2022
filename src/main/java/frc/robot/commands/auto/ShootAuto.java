package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.CargoUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto(
      boolean doesCalculateSpeed,
      boolean isFront,
      double waitTime,
      BooleanSupplier waitCondition,
      double outtakeArmPosition,
      double shooterWheelOuttakeSpeed
    ) {
      this(
        doesCalculateSpeed,
        isFront,
        waitTime,
        waitCondition,
        outtakeArmPosition,
        shooterWheelOuttakeSpeed,
        Robot.belt,
        Robot.arm,
        Robot.shooter
      );
    }

    public ShootAuto(
          boolean doesCalculateSpeed,
          boolean isFront,
          double waitTime,
          BooleanSupplier waitCondition,
          double outtakeArmPosition,
          double shooterWheelOuttakeSpeed,
          Belt belt,
          Arm arm,
          Shooter shooter
    ) {
      addRequirements(shooter, arm, belt);
      addCommands(
        new InstantCommand(() -> CargoUtil.enableAll()),
        parallel(
          new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
          sequence(
            new InstantCommand(() -> CargoUtil.setWheelSpeed(() -> shooterWheelOuttakeSpeed, isFront)),
            new WaitUntilCommand(() -> CargoUtil.isWheelAtSetpoint())
          ),
          sequence(
            new InstantCommand(() -> CargoUtil.setAngle(outtakeArmPosition)),
            new WaitUntilCommand(() -> CargoUtil.isArmAtSetpoint())
          ),
          new WaitUntilCommand(waitCondition),
          new WaitCommand(waitTime)
        ),
        new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kOuttakeSpeed)),
        new WaitCommand(0.6)
      );
    }

    @Override
    public void end(boolean interrupted) {
        CargoUtil.disableShiitake();
    }
}
