package frc.robot.controls;

import controllers.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.Intake;
import frc.robot.commands.PositionArm;
import frc.robot.commands.Shoot;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;
import frc.robot.util.DriveMode;
import frc.robot.util.ShooterMethods;

public class Driver {

  private static PistolController controller = new PistolController(new Joystick(JoyConstants.kDriverJoy));

  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(DriveConstants.kSlewRate);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(DriveConstants.kSlewRate);

  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();
  
  // sets default drive mode
  private static DriveMode driveMode = DriveMode.ARCADE;

  // driver buttons
  public static void configureButtonBindings() {
    controller.getButtons().frontSwitchTop().whenPressed(
        () -> setDriveMode(DriveMode.PROPORTIONAL));
    controller.getButtons().backSwitchTop().whenPressed(

        () -> setDriveMode(DriveMode.ARCADE));
    controller.getButtons().backSwitchBottom().whenHeld(new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kFrontOuttakeFarPos, true, Constants.kIsRedAlliance));
    controller.getButtons().backSwitchBottom().whenReleased(new SequentialCommandGroup(
      new PositionArm(cargoConstants.kFrontOuttakeFarPos),
      new InstantCommand(() -> ShooterMethods.disableShiitake())
    ));
    controller.getButtons().frontSwitchBottom().whileHeld(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));
    controller.getButtons().bottomButton().whenPressed(new ConditionalCommand(
      new Shoot(cargoConstants.kFrontOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true),
      new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true),
      ShooterMethods::isArmFront
    ));
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

}