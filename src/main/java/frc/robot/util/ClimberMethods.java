package frc.robot.util;

import frc.robot.Robot;

public class ClimberMethods {
  public static void setExtension(double inches) {
    Robot.extenderL.setGoal(inches);
    Robot.extenderR.setGoal(inches);
  }

  public static void setAngle(double angle) {
    Robot.rotatorL.setGoal(angle);
    Robot.rotatorR.setGoal(angle);
  }

  public static void setRotatorOutput(double pow) {
    Robot.rotatorL.setOutput(pow);
    Robot.rotatorR.setOutput(pow);
  }

  public static void setExtenderOutput(double pow) {
    Robot.extenderL.setOutput(pow);
    Robot.extenderR.setOutput(pow);
  }

  public static boolean isExtenderAtSetpoint() {
    // if the extenders reached their setpoint, return true
    return Robot.extenderL.reachedSetpoint() && Robot.extenderR.reachedSetpoint();
  }

  public static boolean isRotatorAtSetpoint() {
    // if the rotators reached their setpoint, return true
   // System.out.println("R: " + Robot.mRotatorR.reachedSetpoint());
   // System.out.println("L: " + Robot.mRotatorL.reachedSetpoint());
    return Robot.rotatorR.reachedSetpoint() && Robot.rotatorL.reachedSetpoint();
  }

  public static void enableExtender() {
    Robot.extenderL.enable();
    Robot.extenderR.enable();
  }

  public static void disableExtender() {
    Robot.extenderL.disable();
    Robot.extenderR.disable();
  }

  public static void enableRotator() {
    Robot.rotatorL.enable();
    Robot.rotatorR.enable();
  }

  public static void disableRotator() {
    Robot.rotatorL.disable();
    Robot.rotatorR.disable();
  }

  public static void enableAll() {
    Robot.extenderL.enable();
    Robot.extenderR.enable();
    Robot.rotatorL.enable();
    Robot.rotatorR.enable();
  }

  public static void disableAll() {
    Robot.extenderL.disable();
    Robot.extenderR.disable();
    Robot.rotatorL.disable();
    Robot.rotatorR.disable();
  }

  public static void removeLimiter() {
    Robot.extenderL.removeLimiter();
    Robot.extenderR.removeLimiter();
  }

  public static void enableLimiter() {
    Robot.extenderL.enableLimiter();
    Robot.extenderR.enableLimiter();
  }
}
