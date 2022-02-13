package frc.robot.util;

import frc.robot.RobotContainer;
import frc.robot.robotConstants.extenderArm.TraversoExtenderArmConstants;

public class ClimbMethods {

  public void extenderMaxExtension() {
    RobotContainer.m_extenderL.set(TraversoExtenderArmConstants.kExtenderMaxArmLength); // in inches
    RobotContainer.m_extenderR.set(TraversoExtenderArmConstants.kExtenderMaxArmLength); // in inches
  }

  public void extenderMaxCompression() {
    RobotContainer.m_extenderL.set(0);
    RobotContainer.m_extenderR.set(0);
  }

  public void extenderWhenAtBar() {
    RobotContainer.m_extenderL.set(6); // in inches
    RobotContainer.m_extenderR.set(6); // in inches
  }

  public void extenderHardExtend(double inches) {
    RobotContainer.m_extenderL.set(inches);
    RobotContainer.m_extenderR.set(inches);
  }

  public void rotatorResetAngle() {
    RobotContainer.m_climbArmL.set(90); // in degrees
    RobotContainer.m_climbArmR.set(90); // in degrees
  }

  public void rotatorAngleToBar() {
    RobotContainer.m_climbArmL.set(33); // in degrees
    RobotContainer.m_climbArmR.set(33); // in degrees
  }

  public void rotatorWhenExtendedToBar() {
    RobotContainer.m_climbArmL.set(29); // in degrees
    RobotContainer.m_climbArmR.set(29); // in degrees
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
