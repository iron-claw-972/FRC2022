package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.ShooterMethods;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto(
          boolean doesCalculateSpeed,
          boolean isFront,
          double waitTime,
          BooleanSupplier waitCondition,
          double outtakeArmPosition,
          double shooterWheelOuttakeSpeed
    ) {
      addRequirements(Robot.m_shooter, Robot.m_arm, Robot.m_belt);
      addCommands(
        new InstantCommand(() -> ShooterMethods.enableAll()),
        parallel(
          new InstantCommand(() -> ShooterMethods.setBeltPower(Constants.belt.kIntakeSpeed)),
          sequence(
            new InstantCommand(() -> ShooterMethods.setWheelSpeed(() -> shooterWheelOuttakeSpeed, isFront)),
            new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint())
          ),
          sequence(
            new InstantCommand(() -> ShooterMethods.setAngle(outtakeArmPosition)),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint())
          ),
          new WaitUntilCommand(waitCondition),
          new WaitCommand(waitTime)
        ),
        new InstantCommand(() -> ShooterMethods.setBeltPower(Constants.belt.kOuttakeSpeed)),
        new WaitCommand(0.6)
      );
    }

    @Override
    public void end(boolean interrupted) {
        ShooterMethods.disableShiitake();
    }
}
