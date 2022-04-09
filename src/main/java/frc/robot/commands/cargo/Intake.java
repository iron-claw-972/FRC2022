package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive.DifferentialDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.CargoUtil;

public class Intake extends SequentialCommandGroup {
  public Intake(double postIntakeArmPosition, boolean doesChaseBall) {
    this(postIntakeArmPosition, doesChaseBall, true);
  }

  public Intake(double postIntakeArmPosition, boolean doesChaseBall, boolean isRedBall) {
    this(postIntakeArmPosition, doesChaseBall, isRedBall, true);
  }

  public Intake(double postIntakeArmPosition, boolean doesChaseBall, boolean isRedBall, boolean doesCheckBall) {
    this(postIntakeArmPosition, doesChaseBall, isRedBall, doesCheckBall, Robot.shooter, Robot.arm, Robot.belt, Robot.ll, Robot.drive, Robot.ballDetection);
  }

  public Intake(double postIntakeArmPosition, boolean doesChaseBall, boolean isRedBall, boolean doesCheckBall, Shooter shooter, Arm arm, Belt belt, Limelight limelight, Drivetrain drive, BallDetection ballDetection) {
    addRequirements(shooter, arm, belt, limelight, drive, ballDetection);
    addCommands(
        // Set pipeline early to account for network latency
        new InstantCommand(() -> limelight.setBallPipeline(isRedBall)),

        new InstantCommand(() -> CargoUtil.enableAll()),

        // new ConditionalCommand(
        //   new WaitUntilCommand(() -> !ShooterMethods.isBallContained()),
        //   new DoNothing(), 
        //   () -> doesCheckBall
        // ),

        //System.exit(0);
        // Spin up wheel, belt, set angle, and start ball chase simultaneously
        parallel(
          new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.setAngle(Constants.arm.kIntakePos)),
          new ConditionalCommand(
            new ChaseBall(isRedBall),
            new DifferentialDrive(drive),
            () -> doesChaseBall
          )
        ),
        new InstantCommand(() -> Robot.drive.tankDriveVolts(0, 0)),

        // new DoNothing()
        new ConditionalCommand(
          sequence(
            // Bring arm back up and stop intake when we have the ball
            new WaitUntilCommand(() -> CargoUtil.isBallContained()),
            new InstantCommand(() -> CargoUtil.disableShiitake()),
            new InstantCommand(() -> CargoUtil.setAngle(postIntakeArmPosition)),
            new WaitUntilCommand(() -> CargoUtil.isArmAtSetpoint()).withTimeout(1)
          ),
          new DoNothing(), 
          () -> doesCheckBall
        )
    );
  }

  @Override
  public void end(boolean interrupted) {
    CargoUtil.disableShiitake();
    // limelight.setDriverPipeline();
    Robot.drive.tankDriveVolts(0, 0);
  }
}
