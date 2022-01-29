package frc.robot.controls;

import java.sql.Driver;

import frc.robot.Constants.*;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Functions {
  
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

  public static double slewCalculate(double rate, double input ){
    return new SlewRateLimiter(rate).calculate(input);
  }
}
