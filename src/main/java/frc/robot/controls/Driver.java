package frc.robot.controls;


import controllers.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.cargoCommands.AlignToUpperHub;
import frc.robot.commands.cargoCommands.Intake;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.commands.cargoCommands.Shoot;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.MarinusBeltConstants;
import frc.robot.robotConstants.shooterWheel.MarinusCargoShooterConstants;
import frc.robot.util.DriveMode;

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
    // Align to hub front
    controller.getButtons().frontSwitchBottom().whileHeld(new SequentialCommandGroup(
      new PositionArm(cargoConstants.kFrontLimelightScanPos),
      new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive)
    ));

    // Align to hub back
    controller.getButtons().backSwitchBottom().whileHeld(new SequentialCommandGroup(
      new PositionArm(cargoConstants.kBackLimelightScanPos),
      new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive)
    ));

    // Intake w/ ball chase for our color
    controller.getButtons().frontSwitchTop().whenHeld(new Intake(RobotContainer.cargoConstants.kFrontLimelightScanPos, true, Constants.kIsRedAlliance));
    controller.getButtons().frontSwitchTop().whenReleased(new PositionArm(cargoConstants.kFrontLimelightScanPos));

    // Intake w/ ball chase for opponent color
    controller.getButtons().backSwitchTop().whenHeld(new Intake(RobotContainer.cargoConstants.kFrontLimelightScanPos, true, !Constants.kIsRedAlliance));
    controller.getButtons().backSwitchTop().whenReleased(new PositionArm(cargoConstants.kFrontLimelightScanPos));
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
