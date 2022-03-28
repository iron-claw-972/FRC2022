package frc.robot.commands.cargo;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.util.ShooterMethods;

public class Shoot extends SequentialCommandGroup {
    public Shoot(boolean doesCalculateSpeed, boolean doesAlign, boolean isFront) {
        this(doesCalculateSpeed, doesAlign, isFront, Double.NaN, Double.NaN);
    }

    public Shoot(
          boolean doesCalculateSpeed,
          boolean doesAlign,
          boolean isFront,
          double outtakeArmPosition,
          double shooterWheelOuttakeSpeed
    ) {
      addRequirements(Robot.m_shooter, Robot.m_arm, Robot.m_belt, Robot.m_limelight);
      addCommands(
        // Start spin up before so PID has less work to do
        new InstantCommand(() -> ShooterMethods.setWheelRPM(-2000)),

        // Set hub pipeline early to account for network latency
        new InstantCommand(() -> Robot.m_limelight.setUpperHubPipeline()),

        // Don't spin shooting wheels until ball is confirmed to be in shooter
        new InstantCommand(() -> ShooterMethods.setBeltPower(Constants.belt.kIntakeSpeed)),
        new WaitUntilCommand(() -> ShooterMethods.isBallContained()).withTimeout(0.4),

        new InstantCommand(() -> ShooterMethods.enableAll()),

        parallel(
          sequence(
            // Spin up wheels to optimal velocity when the limelight gets the optimal angle
            new ConditionalCommand(
              sequence(
                new WaitUntilCommand(() -> GetDistance.isFinished),
                new InstantCommand(() -> ShooterMethods.setWheelSpeed(() -> GetDistance.optimalVelocity, isFront))
              ),
              new InstantCommand(() -> ShooterMethods.setWheelRPM(shooterWheelOuttakeSpeed)),
              () -> doesCalculateSpeed
            ),

            new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint())
          ),

          // Limelight stuff
          sequence(
            // Get arm to limelight angle
            new ConditionalCommand(
              new PositionArm((isFront ? Constants.arm.kFrontLimelightScanPos : Constants.arm.kBackLimelightScanPos)),
              new DoNothing(), 
              () -> doesCalculateSpeed || doesAlign
            ),

            // // Align using limelight
            new ConditionalCommand(
              new AlignToUpperHub(Robot.m_limelight, Robot.m_drive).withTimeout(2),
              new DoNothing(),
              () -> doesAlign
            ),

            //  Calculate distance and determines optimal shooting angle and velocity and
            // set to actual shooting angle
            new ConditionalCommand(
              sequence(
                new GetDistance(Robot.m_limelight, Robot.m_arm),
                new PositionArmOptimal()
              ),
              new PositionArm(outtakeArmPosition),
              () -> doesCalculateSpeed
            )
          )
        ),

        // // Wait until both arm and wheels are at setpoint
        new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint() && ShooterMethods.isArmAtSetpoint()),

        // Spin belts to outtake ball
        new InstantCommand(() -> ShooterMethods.setBeltPower(Constants.belt.kOuttakeSpeed)),
        new WaitCommand(0.4)
      );
    }

    @Override
    public void end(boolean interrupted) {
      ShooterMethods.disableShiitake();
      // Robot.m_limelight.setDriverPipeline();
      GetDistance.isFinished = false; // Reset finished condition
    }
}
