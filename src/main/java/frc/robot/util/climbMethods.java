package frc.robot.util;

import java.util.function.BooleanSupplier;

import frc.robot.RobotContainer;

public class ClimbMethods {
  public static void extenderHardExtend(double inches) {
    RobotContainer.m_extenderL.set(inches);
    RobotContainer.m_extenderR.set(inches);
  }

  public static void rotatorHardAngle(double angle) {
    RobotContainer.m_climbArmL.set(angle);
    RobotContainer.m_climbArmR.set(angle);
  }

  public static boolean extenderSetCheck() {
    // if the extenders reached their setpoint, return true
    return RobotContainer.m_extenderL.reachedSetpoint() && RobotContainer.m_extenderR.reachedSetpoint();
  }

  public static boolean rotatorSetCheck() {
    // if the rotators reached their setpoint, return true
    return RobotContainer.m_climbArmR.reachedSetpoint() && RobotContainer.m_climbArmL.reachedSetpoint();
  }
}
