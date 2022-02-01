package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import frc.robot.util.DriveMode;

public class Driver{

  private static GameC controller = new GameC(new Joystick(JoyConstants.kDriverJoy));

  private static double lowSensThrottle = 0.2 , lowSensTurn = 0.4, highSensThrottle = 1, highSensTurn = 0.5;
  private static double sensThrottle = lowSensThrottle, sensTurn = lowSensTurn;
  
  //sets defult drive mode
  private static DriveMode driveMode = DriveMode.ARCADE;

  //driver buttons
  public static void configureButtonBindings() {
    controller.Button.B().whenPressed(() -> modSensitivity());
    controller.Button.A().whenPressed(() -> modDrive());
  }
  
  public static double getThrottleValue() {
    //put any proses in any order of the driver's choseing
    return
      Functions.slewCalculate(5,
      -getRawThrottleValue()*sensThrottle // Controllers y-axes are natively up-negative, down-positive
    );
  }

  public static double getTurnValue() {
    //Right is Positive left is negitive
    return
      Functions.slewCalculate(5,
      getRawThrottleValue()*sensTurn
    );
  }

  public static void modSensitivity(){
    if (sensThrottle == highSensThrottle) {
      sensThrottle = lowSensThrottle;
      sensTurn = lowSensTurn;
      System.out.println("sensitivity changed to low");
    } else {
      sensThrottle = highSensThrottle;
      sensTurn = highSensTurn;
      System.out.println("sensitivity changed to high");
    }
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