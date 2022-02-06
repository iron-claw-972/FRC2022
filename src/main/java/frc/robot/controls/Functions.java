package frc.robot.controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;

public class Functions {
  
  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(DriveConstants.kSpeedSlewRateLimit);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(DriveConstants.kRotationSlewRateLimit);

  /*
  public static Functions controls;
  public static Functions getControls(){
    return controls;
  }
  */

  /**
   * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
   * linear from (deadband, 0) to (1,1)
   * 
   * @param input    The input value to rescale
   * @param deadband The deadband
   * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
   */
  public static double deadband(double deadband, double input) {
    if (Math.abs(input) <= deadband) {
        return 0;
    } else if (Math.abs(input) == 1) {
        return input;
    } else {
        return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
    }
  }

  //is an exponetional function that maintains positive or negitive
  public static double expoMS(double exponent, double base){
    //weird stuff will hapen if you don't put a number > 0
    double finVal = Math.pow(Math.abs(base),exponent);
    if (base < 0) {
      finVal *= -1;
    }
    return finVal;
  }

  public static double slewCalculateThrottle(double input ){
    return slewThrottle.calculate(input);
  }

  public static double slewCalculateTurn(double input ){
    return slewTurn.calculate(input);
  }
}
