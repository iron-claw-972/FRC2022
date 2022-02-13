package frc.robot.util;

import frc.robot.RobotContainer;

public class climbMethods {
  public void extenderHardExtend(double inches) {
    RobotContainer.m_extenderL.set(inches);
    RobotContainer.m_extenderR.set(inches);
  }

  public void rotatorHardAngle(double angle) {
    RobotContainer.m_climbArmL.set(angle);
    RobotContainer.m_climbArmR.set(angle);
  }

  public boolean extenderSetCheck() {
    // if the extenders reached their setpoint, return true
    return RobotContainer.m_extenderL.reachedSetpoint() && RobotContainer.m_extenderR.reachedSetpoint();
  }

  public boolean rotatorSetCheck() {
    // if the rotators reached their setpoint, return true
    return RobotContainer.m_climbArmR.reachedSetpoint() && RobotContainer.m_climbArmL.reachedSetpoint();
  }
}
