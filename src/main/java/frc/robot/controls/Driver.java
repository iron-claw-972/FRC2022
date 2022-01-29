package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.setup.controllers.*;
import frc.robot.controls.Functions;

public class Driver{

  public static GameC controller = new GameC(new Joystick(JoyConstants.kDriverJoy));

  public static double lowSensThrottle = 0.2 , lowSensTurn = 0.4, highSensThrottle = 1, highSensTurn = 0.5;
  public static double sensThrottle = lowSensThrottle, sensTurn = lowSensTurn;
  
  //sets defult drive mode
  public static String driveMode = "arcade";

  //driver buttons
  public static void configureButtonBindings() {
    controller.Button.B().whenPressed(() -> modSensitivity());
    controller.Button.A().whenPressed(() -> modDrive());
  }
  
  public static double getThrottleValue() {
    //put any proses in any order of the driver's choseing
    return
      Functions.slewCalculate(5,
      -getRawThrottleValue() // Controllers y-axes are natively up-negative, down-positive
    );
  }

  public static double getTurnValue() {
    //Right is Positive left is negitive
    return
      Functions.slewCalculate(5,
      getRawThrottleValue()
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
    System.out.println("modding drive");
    if (driveMode == "arcade") {
      driveMode = "prop";
    }else if (driveMode == "prop") {
    //   driveMode = "shift";
    // }else if (driveMode == "shift") {
      driveMode = "arcade";
    }
  }
  
  //checks drive mode
  public static boolean isDrive(String drive){
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

  }