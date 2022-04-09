package frc.robot.commands.cargo;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class Shoot extends SequentialCommandGroup {
    public Shoot(boolean doesCalculateSpeed, boolean doesAlign, boolean isFront) {
        this(doesCalculateSpeed, doesAlign, isFront, Double.NaN, Double.NaN, Robot.shooter, Robot.arm, Robot.belt, Robot.ll, Robot.drive, Robot.ballDetection);
    }

    public Shoot(boolean doesCalculateSpeed, boolean doesAlign, boolean isFront, Shooter shooter, Arm arm, Belt belt, Limelight limelight, Drivetrain drive, BallDetection ballDetection) {
        this(doesCalculateSpeed, doesAlign, isFront, Double.NaN, Double.NaN, shooter, arm, belt, limelight, drive, ballDetection);
    }

    public Shoot(boolean doesCalculateSpeed, boolean doesAlign, boolean isFront, double outtakeArmPosition, double shooterWheelOuttakeSpeed) {
        this(doesCalculateSpeed, doesAlign, isFront, outtakeArmPosition, shooterWheelOuttakeSpeed, Robot.shooter, Robot.arm, Robot.belt, Robot.ll, Robot.drive, Robot.ballDetection);
    }

    public Shoot(
          boolean doesCalculateSpeed,
          boolean doesAlign,
          boolean isFront,
          double outtakeArmPosition,
          double shooterWheelOuttakeSpeed,
          Shooter shooter,
          Arm arm,
          Belt belt,
          Limelight limelight,
          Drivetrain drive,
          BallDetection ballDetection
    ) {
      addRequirements(shooter, arm, belt, limelight, drive, ballDetection);
      addCommands(
        // Start spin up before so PID has less work to do
        new InstantCommand(() -> CargoUtil.setWheelRPM(-2000)),

        // Set hub pipeline early to account for network latency
        new InstantCommand(() -> limelight.setUpperHubPipeline()),

        // Don't spin shooting wheels until ball is confirmed to be in shooter
        new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
        new InstantCommand(() -> CargoUtil.enableBelt()),
        new WaitUntilCommand(() -> CargoUtil.isBallContained()).withTimeout(0.4),

        new InstantCommand(() -> CargoUtil.enableAll()),
      
        //like parallel parking :)
        parallel(
          sequence(
            // Spin up wheels to optimal velocity when the limelight gets the optimal angle
            new ConditionalCommand(
              sequence(
                new WaitUntilCommand(() -> GetDistance.isFinished),
                new InstantCommand(() -> CargoUtil.setWheelSpeed(() -> GetDistance.optimalVelocity, isFront))
              ),
              new InstantCommand(() -> CargoUtil.setWheelRPM(shooterWheelOuttakeSpeed)),
              () -> doesCalculateSpeed
            ),

            new WaitUntilCommand(() -> CargoUtil.isWheelAtSetpoint())
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
              new AlignToUpperHub(limelight, drive).withTimeout(2),
              new DoNothing(),
              () -> doesAlign
            ),

            //  Calculate distance and determines optimal shooting angle and velocity and
            // set to actual shooting angle
            deadline(
              new ConditionalCommand(
                sequence(
                  new GetDistance(limelight, arm),
                  new PositionArmOptimal()
                ),
                new PositionArm(outtakeArmPosition),
                () -> doesCalculateSpeed
              ),
              new InstantCommand(() -> drive.tankDriveVolts(0, 0)).perpetually()
            )
          )
        ),

        // // Wait until both arm and wheels are at setpoint
        deadline(
          new WaitUntilCommand(() -> CargoUtil.isWheelAtSetpoint() && CargoUtil.isArmAtSetpoint()),
          new InstantCommand(() -> drive.tankDriveVolts(0, 0)).perpetually()
        ),

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
