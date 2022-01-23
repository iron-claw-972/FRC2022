package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.kJoy;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;

import frc.robot.setup.controllers.*;
import frc.robot.setup.controllers.GameC.*;

public class Controls {
  public static Drivetrain m_drive = new Drivetrain();


  GameC driver = new GameC(new Joystick(kJoy.kDriverJoy));
  // static Joystick operator = new Joystick(kJoy.kOperatorJoy);
  
  public void configureButtonBindings() {
    //driver buttons
  
    // operator_B.whenPressed(() -> m_drive.modSensitivity());
    driver.Button.A().whenPressed(() -> m_drive.modDrive());


    //Operator buttons
    
  }

  // driver val
  public double getThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive. This method
    // corrects that by returning the opposite of the y-value
    return 0; //-driver.getRawAxis(1);
  }

  public double getTurnValue() {
      return driver.JoystickAxis.rightX();
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
