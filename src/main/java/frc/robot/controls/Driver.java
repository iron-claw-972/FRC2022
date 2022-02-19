package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;
import frc.robot.util.DriveMode;

public class Driver {

  private static GameController controller = new GameController(new Joystick(JoyConstants.kDriverJoy));
  
  // sets default drive mode
  private static DriveMode driveMode = DriveMode.ARCADE;

  // driver buttons
  public static void configureButtonBindings() {
    controller.getButtons().B().whenPressed(
        () -> setDriveMode(DriveMode.PROPORTIONAL));
    controller.getButtons().A().whenPressed(
        () -> setDriveMode(DriveMode.ARCADE));
  }
  
  public static double getThrottleValue() {
    // put any processes in any order of the driver's choosing
    // Controllers y-axes are natively up-negative, down-positive
    return Functions.slewCalculateThrottle(Functions.deadband(JoyConstants.kDeadband, getRawThrottleValue()));
  }

  public static double getTurnValue() {
    // right is positive; left is negative
    return Functions.slewCalculateTurn(Functions.deadband(JoyConstants.kDeadband, getRawTurnValue()));
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
    return controller.getJoystickAxis().leftY();
  }

  public static double getRawTurnValue() {
    // Right is Positive left is negative
    return controller.getJoystickAxis().rightX();
  }

  public static DriveMode getDriveMode() {
    return driveMode;
  }

}