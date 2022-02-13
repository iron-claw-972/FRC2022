package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;
import frc.robot.util.DriveMode;

public class Driver {

  private static PistolController controller = new PistolController(new Joystick(JoyConstants.kDriverJoy));
  
  // sets default drive mode
  private static DriveMode driveMode = DriveMode.ARCADE;

  // driver buttons
  public static void configureButtonBindings() {
    controller.getButtons().backSwitchTop().whenPressed(
        () -> RobotContainer.m_drive.setMaxOutput(DriveConstants.kSlowSpeed), RobotContainer.m_drive);
    controller.getButtons().frontSwitchTop().whenPressed(
        () -> RobotContainer.m_drive.setMaxOutput(1.0), RobotContainer.m_drive);
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
    return controller.TriggerAxis();
  }

  public static double getRawTurnValue() {
    // Right is Positive left is negative
    return controller.WheelAxis();
  }

  public static DriveMode getDriveMode() {
    return driveMode;
  }

}