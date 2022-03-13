package frc.robot.util;

import frc.robot.RobotContainer;

public class ClimberMethods {
  public static void setExtension(double inches) {
    RobotContainer.m_extenderL.set(inches);
    RobotContainer.m_extenderR.set(inches);
  }

  public static void setAngle(double angle) {
    RobotContainer.m_climbRotatorL.setGoal(angle);
    RobotContainer.m_climbRotatorR.setGoal(angle);
  }

  public static void setRotatorOutput(double pow) {
    RobotContainer.m_climbRotatorL.setOutput(pow);
    RobotContainer.m_climbRotatorR.setOutput(pow);
  }

  public static void setExtenderOutput(double pow) {
    RobotContainer.m_extenderR.setOutput(pow);
    RobotContainer.m_extenderL.setOutput(pow);
  }

  public static boolean extenderLimitSwitchRisingEdge() {
    return RobotContainer.m_extenderL.limitSwitchRisingEdge() && RobotContainer.m_extenderR.limitSwitchRisingEdge();
  }

  public static boolean isExtenderAtSetpoint() {
    // if the extenders reached their setpoint, return true
    return RobotContainer.m_extenderL.reachedSetpoint() && RobotContainer.m_extenderR.reachedSetpoint();
  }

  public static boolean isRotatorAtSetpoint() {
    // if the rotators reached their setpoint, return true
    return RobotContainer.m_climbRotatorR.reachedSetpoint() && RobotContainer.m_climbRotatorL.reachedSetpoint();
  }

  public static void enableExtender() {
    RobotContainer.m_extenderL.enable();
    RobotContainer.m_extenderR.enable();
  }

  public static void disableExtender() {
    RobotContainer.m_extenderL.disable();
    RobotContainer.m_extenderR.disable();
  }

  public static void enableRotator() {
    RobotContainer.m_climbRotatorL.enable();
    RobotContainer.m_climbRotatorR.enable();
  }

  public static void disableRotator() {
    RobotContainer.m_climbRotatorL.disable();
    RobotContainer.m_climbRotatorR.disable();
  }

  public static void enableAll() {
    RobotContainer.m_extenderL.enable();
    RobotContainer.m_extenderR.enable();
    RobotContainer.m_climbRotatorL.enable();
    RobotContainer.m_climbRotatorR.enable();
  }

  public static void disableAll() {
    RobotContainer.m_extenderL.disable();
    RobotContainer.m_extenderR.disable();
    RobotContainer.m_climbRotatorL.disable();
    RobotContainer.m_climbRotatorR.disable();
  }

  public static void removeLimiter() {
    RobotContainer.m_extenderL.removeLimiter();
    RobotContainer.m_extenderR.removeLimiter();
  }

  public static void enableLimiter() {
    RobotContainer.m_extenderL.enableLimiter();
    RobotContainer.m_extenderR.enableLimiter();
  }

  public static void zeroExtender() {
    RobotContainer.m_extenderR.zero();
    RobotContainer.m_extenderL.zero();
  }
}
