package frc.robot.util;

import frc.robot.RobotContainer;

public class ClimberMethods {
  public static void setExtension(double inches) {
    RobotContainer.m_extenderL.set(inches);
    RobotContainer.m_extenderR.set(inches);
  }

  public static void setAngle(double angle) {
    RobotContainer.m_climbArmL.setGoal(angle);
    RobotContainer.m_climbArmR.setGoal(angle);
  }

  public static boolean isExtenderAtSetpoint() {
    // if the extenders reached their setpoint, return true
    return RobotContainer.m_extenderL.reachedSetpoint() && RobotContainer.m_extenderR.reachedSetpoint();
  }

  public static boolean isRotatorAtSetpoint() {
    // if the rotators reached their setpoint, return true
    return RobotContainer.m_climbArmR.reachedSetpoint() && RobotContainer.m_climbArmL.reachedSetpoint();
  }
}
