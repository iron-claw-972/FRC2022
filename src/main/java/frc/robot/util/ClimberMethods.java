package frc.robot.util;

import frc.robot.RobotContainer;

public class ClimberMethods {
  public static void setExtension(double inches) {
    RobotContainer.m_extenderL.set(inches);
    RobotContainer.m_extenderR.set(inches);
  }

  public static void setAngle(double angle) {
    RobotContainer.m_rotatorL.setGoal(angle);
    RobotContainer.m_rotatorR.setGoal(angle);
  }

  public static boolean isExtenderAtSetpoint() {
    // if the extenders reached their setpoint, return true
    return RobotContainer.m_extenderL.reachedSetpoint() && RobotContainer.m_extenderR.reachedSetpoint();
  }

  public static boolean isRotatorAtSetpoint() {
    // if the rotators reached their setpoint, return true
    return RobotContainer.m_rotatorR.reachedSetpoint() && RobotContainer.m_rotatorL.reachedSetpoint();
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
    RobotContainer.m_rotatorL.enable();
    RobotContainer.m_rotatorR.enable();
  }

  public static void disableRotator() {
    RobotContainer.m_rotatorL.disable();
    RobotContainer.m_rotatorR.disable();
  }
}
