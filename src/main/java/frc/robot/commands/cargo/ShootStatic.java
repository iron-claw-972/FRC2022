package frc.robot.commands.cargo;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.CargoUtil;

public class ShootStatic extends SequentialCommandGroup {

    public ShootStatic() {
        this(Robot.shooter, Robot.arm, Robot.belt, Robot.ll, Robot.drive, Robot.ballDetection);
    }

    public ShootStatic(
          Shooter shooter,
          Arm arm,
          Belt belt,
          Limelight limelight,
          Drivetrain drive,
          BallDetection ballDetection
    ) {
      addRequirements(shooter, arm, belt, limelight, drive, ballDetection);
      addCommands(
        new InstantCommand(() -> CargoUtil.disableShiitake()),
        // Start spin up before so PID has less work to do
        new InstantCommand(() -> CargoUtil.setWheelRPM(-2000)),

        // Set hub pipeline early to account for network latency
        new InstantCommand(() -> limelight.setUpperHubPipeline()),

        parallel(
          sequence(
            // Don't spin shooting wheels until ball is confirmed to be in shooter
            new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.enableBelt()),
            new WaitUntilCommand(() -> CargoUtil.isBallContained()).withTimeout(0.4),

            new InstantCommand(() -> CargoUtil.enableAll())
          ),
          new PositionArm(Constants.arm.kOptimalBackShootingPos)
        ),
        new AlignToUpperHub(),
        new GetDistance(false),
        new WaitUntilCommand(() -> GetDistance.isFinished),
        new InstantCommand(() -> CargoUtil.setWheelSpeed(() -> GetDistance.optimalVelocity, false)),
        new WaitUntilCommand(() -> CargoUtil.isWheelAtSetpoint() && CargoUtil.isArmAtSetpoint()),

        // deadline(
        //   //  && Robot.ll.hasValidTarget() && AlignToUpperHub.alignPID.atSetpoint() && GetDistance.isFinished),
          
          
        // ),

        // Spin belts to outtake ball
        new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kOuttakeSpeed)),
        new WaitCommand(0.4)
      );
    }

    @Override
    public void end(boolean interrupted) {
      CargoUtil.disableShiitake();
      // Robot.m_limelight.setDriverPipeline();
      GetDistance.isFinished = false; // Reset finished condition
    }
}
