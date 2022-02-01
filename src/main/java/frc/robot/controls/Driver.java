package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DriveMode;

public class Driver{

  private static GameC controller = new GameC(new Joystick(JoyConstants.kDriverJoy));
  
  //sets defult drive mode
  private static DriveMode driveMode = DriveMode.ARCADE;

  //driver buttons
  public static void configureButtonBindings() {
    controller.Button.B().whenPressed(
        () -> Drivetrain.getInstance().setMaxOutput(DriveConstants.kSlowSpeed), Drivetrain.getInstance());
    controller.Button.A().whenPressed(
        () -> Drivetrain.getInstance().setMaxOutput(1.0), Drivetrain.getInstance());
  }
  
  public static double getThrottleValue() {
    // put any proses in any order of the driver's choseing
    // Controllers y-axes are natively up-negative, down-positive
    return Functions.slewCalculate(5, -getRawThrottleValue());
  }

  public static double getTurnValue() {
    // right is positive; left is negative
    return Functions.slewCalculate(5, getRawThrottleValue());
  }
  
  // cyles drive mode
  public static void modDrive(){
    driveMode = driveMode.next();
  }
  
  //checks drive mode
  public static boolean isDrive(DriveMode drive){
    return (driveMode == drive);
  }

  public static double getRawThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive
    return controller.JoystickAxis.leftY();
  }

  public static double getRawTurnValue() {
    //Right is Positive left is negitive
    return controller.JoystickAxis.rightX();
  }

  public static DriveMode getDriveMode() {
    return driveMode;
  }

}