package frc.robot.util;

import frc.robot.Robot;

public class ClimberMethods {
  public static void setExtension(double inches) {
    Robot.m_extenderL.set(inches);
    Robot.m_extenderR.set(inches);
  }

  public static void setAngle(double angle) {
    Robot.m_rotatorL.setGoal(angle);
    Robot.m_rotatorR.setGoal(angle);
  }

  public static void setRotatorOutput(double pow) {
    Robot.m_rotatorL.setOutput(pow);
    Robot.m_rotatorR.setOutput(pow);
  }

  public static void setExtenderOutput(double pow) {
    Robot.m_extenderL.setOutput(pow);
    Robot.m_extenderR.setOutput(pow);
  }

  public static boolean isExtenderAtSetpoint() {
    // if the extenders reached their setpoint, return true
    return Robot.m_extenderL.reachedSetpoint() && Robot.m_extenderR.reachedSetpoint();
  }

  public static boolean isRotatorAtSetpoint() {
    // if the rotators reached their setpoint, return true
   // System.out.println("R: " + Robot.mRotatorR.reachedSetpoint());
   // System.out.println("L: " + Robot.mRotatorL.reachedSetpoint());
    return Robot.m_rotatorR.reachedSetpoint() && Robot.m_rotatorL.reachedSetpoint();
  }

  public static void enableExtender() {
    Robot.m_extenderL.enable();
    Robot.m_extenderR.enable();
  }

  public static void disableExtender() {
    Robot.m_extenderL.disable();
    Robot.m_extenderR.disable();
  }

  public static void enableRotator() {
    Robot.m_rotatorL.enable();
    Robot.m_rotatorR.enable();
  }

  public static void disableRotator() {
    Robot.m_rotatorL.disable();
    Robot.m_rotatorR.disable();
  }

  public static void enableAll() {
    Robot.m_extenderL.enable();
    Robot.m_extenderR.enable();
    Robot.m_rotatorL.enable();
    Robot.m_rotatorR.enable();
  }

  public static void disableAll() {
    Robot.m_extenderL.disable();
    Robot.m_extenderR.disable();
    Robot.m_rotatorL.disable();
    Robot.m_rotatorR.disable();
  }

  public static void removeLimiter() {
    Robot.m_extenderL.removeLimiter();
    Robot.m_extenderR.removeLimiter();
  }

  public static void enableLimiter() {
    Robot.m_extenderL.enableLimiter();
    Robot.m_extenderR.enableLimiter();
  }
}
