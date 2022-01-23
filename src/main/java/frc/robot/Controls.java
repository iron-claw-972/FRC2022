package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.kJoy;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;

import frc.robot.setup.controllers.*;
import frc.robot.setup.controllers.GameC.*;

public class Controls {
  public static Drivetrain m_drive = new Drivetrain();


  static Joystick driver = new Joystick(kJoy.kDriverJoy);
  static Joystick operator = new Joystick(kJoy.kOperatorJoy);
  
  public Controls(){

  }
  
  public void configureButtonBindings() {
    //driver controls
  
    // operator_B.whenPressed(() -> m_drive.modSensitivity());
    GameC.Button.A(driver).whenPressed(() -> m_drive.modDrive());
    
    //Operator controls
    
  }
  
  // driver val
  public static double getThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive. This method
    // corrects that by returning the opposite of the y-value
    return -driver.getRawAxis(1);
  }

  public static double getTurnValue() {
      // 4 represents left/right axis on the right joystick
      return driver.getRawAxis(4);
      // return -driver.getRawAxis(0);
  }

  /**
   * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
   * linear from (deadband, 0) to (1,1)
   * 
   * @param input    The input value to rescale
   * @param deadband The deadband
   * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
   */
  public static double deadband(double input, double deadband) {
    if (Math.abs(input) <= deadband) {
        return 0;
    } else if (Math.abs(input) == 1) {
        return input;
    } else {
        return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
    }
  }

}
