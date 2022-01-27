package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.setup.controllers.*;

public class Controls {
  public static Controls controls;
  public static Controls getControls(){
    return controls;
  }

  public static Drivetrain m_drive = new Drivetrain();


  GameC driver = new GameC(new Joystick(JoyConstants.kDriverJoy));

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(DriveConstants.kSpeedSlewRateLimit);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRateLimit);

  // static Joystick operator = new Joystick(kJoy.kOperatorJoy);
  
  public void configureButtonBindings() {
    //driver buttons
    driver.Button.B().whenPressed(() -> m_drive.modSensitivity());
    driver.Button.A().whenPressed(() -> m_drive.modDrive());

    //Operator buttons
    

  }

  // driver val
  public double getThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive. This method
    // corrects that by returning the opposite of the y-value
    return -driver.JoystickAxis.leftY();
  }

  public double getTurnValue() {
    //Right is Positive left is negitive
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

  //is an exponetional function that maintains positive or negitive
  public double expoMS(double base, double exponent){
    //weird stuff will hapen if you don't put a number > 0
    double finVal = Math.pow(Math.abs(base),exponent);
    if (base < 0) {
      finVal *= -1;
    }
    return finVal;
  }
}
