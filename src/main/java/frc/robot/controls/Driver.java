package frc.robot.controls;


import controllers.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.Intake;
import frc.robot.commands.PositionArm;
import frc.robot.commands.Shoot;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.MarinusBeltConstants;
import frc.robot.robotConstants.shooterWheel.MarinusCargoShooterConstants;
import frc.robot.util.DriveMode;
import frc.robot.util.ShooterMethods;

public class Driver {

  private static PistolController controller = new PistolController(new Joystick(JoyConstants.kDriverJoy));

  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(DriveConstants.kSlewRate);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(DriveConstants.kSlewRate);

  public static MarinusCargoRotatorConstants cargoConstants = new MarinusCargoRotatorConstants();
  public static MarinusBeltConstants beltConstants = new MarinusBeltConstants();
  public static MarinusCargoShooterConstants wheelConstants = new MarinusCargoShooterConstants();
  
  // sets default drive mode
  private static DriveMode driveMode = DriveMode.ARCADE;

  // driver buttons
  public static void configureButtonBindings() {
    controller.getButtons().frontSwitchTop().whenPressed(
        () -> swapDriveMode(DriveMode.PROPORTIONAL , DriveMode.ARCADE));
    controller.getButtons().frontSwitchTop().whenReleased(
        () -> swapDriveMode(DriveMode.PROPORTIONAL , DriveMode.ARCADE));
    controller.getButtons().backSwitchTop().whenPressed(
        () -> swapDriveMode(DriveMode.PROPORTIONAL , DriveMode.ARCADE));

    controller.getButtons().backSwitchBottom().whenHeld(new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kFrontOuttakeFarPos, true, Constants.kIsRedAlliance));
    controller.getButtons().backSwitchBottom().whenReleased(new SequentialCommandGroup(
      new PositionArm(cargoConstants.kFrontOuttakeFarPos),
      new InstantCommand(() -> ShooterMethods.disableShiitake())
    ));

    controller.getButtons().frontSwitchBottom().whenHeld(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));


    controller.getButtons().bottomButton().whenHeld(new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.m_limelight.setUpperHubPipeline()),
      new WaitCommand(0.1),
      new ConditionalCommand(
        new Shoot(cargoConstants.kFrontOuttakeHighPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeFarSpeed, beltConstants.kOuttakeSpeed, false, 0),
        new Shoot(cargoConstants.kBackOuttakeLimelightPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, false, 0),
        ShooterMethods::isArmFront
      )
    ));
    controller.getButtons().bottomButton().whenReleased(new InstantCommand(() -> RobotContainer.m_limelight.setCameraMode(true)));
    
    
    controller.getButtons().backSwitchBottom().whenHeld(new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kFrontOuttakeFarPos, true, Constants.kIsRedAlliance));
    controller.getButtons().frontSwitchBottom().whenHeld(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));
  }
  
  public static double getThrottleValue() {
    // put any processes in any order of the driver's choosing
    // Controllers y-axes are natively up-negative, down-positive
    return slewThrottle.calculate(Functions.deadband(JoyConstants.kDeadband, getRawThrottleValue()));
  }

  public static double getTurnValue() {
    // right is positive; left is negative
    return slewTurn.calculate(Functions.deadband(JoyConstants.kDeadband, getRawTurnValue()));
  }
  
  // sets drive mode
  public static void setDriveMode(DriveMode dm) {
    driveMode = dm;
  }
  
  //checks drive mode
  public static boolean isDrive(DriveMode drive) {
    return (driveMode == drive);
  }

  public static double getRawThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive
    return controller.getTriggerAxis();
  }

  public static double getRawTurnValue() {
    // Right is Positive left is negative
    return controller.getWheelAxis();
  }

  public static DriveMode getDriveMode() {
    return driveMode;
  }
  
  public static void swapDriveMode(DriveMode primary , DriveMode secondary){
    if (driveMode == primary) {
      setDriveMode(secondary);
    } else if (driveMode == secondary){
      setDriveMode(primary);
    } else {
      setDriveMode(primary);
    }
  }

}
