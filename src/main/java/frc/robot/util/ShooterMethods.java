package frc.robot.util;

import frc.robot.RobotContainer;
import frc.robot.subsystems.BallDetection;

public class ShooterMethods {
  // cargo arm methods
  public static void setAngle(double angle) {
    RobotContainer.m_cargoArm.setPosition(angle);
  }

  public static void enableArm() {
    RobotContainer.m_cargoArm.enable();
  }

  public static void disableArm() {
    RobotContainer.m_cargoArm.disable();
  }

  public static boolean isArmAtSetpoint() {
    return RobotContainer.m_cargoArm.reachedSetpoint();
  }
  //

  // belt methods
  public static void setBeltSpeed(double speed) {
    RobotContainer.m_shooterBelt.setOutput(speed);
  }

  public static void enableBelt() {
    RobotContainer.m_shooterBelt.enable();
  }

  public static void disableBelt() {
    RobotContainer.m_shooterBelt.disable();
  }

  public static void stopBelt() {
    RobotContainer.m_shooterBelt.setStop();
  }
  //

  // wheel methods
  public static void setWheelSpeed(double speed) {
    RobotContainer.m_shooterWheel.setSpeed(speed);
  }

  public static void enableWheel() {
    RobotContainer.m_shooterWheel.enable();
  }
  public static void disableWheel() {
    RobotContainer.m_shooterWheel.disable();
  }

  public static boolean isWheelAtSetpoint() {
    return RobotContainer.m_shooterWheel.reachedSetpoint();
  }

  public static void stopWheel() {
    RobotContainer.m_shooterWheel.setStop();
  }
  //

  public static void enableAll() {
    RobotContainer.m_cargoArm.enable();
    RobotContainer.m_shooterBelt.enable();
    RobotContainer.m_shooterWheel.enable();
  }

  public static void disableAll() {
    RobotContainer.m_cargoArm.disable();
    RobotContainer.m_shooterBelt.disable();
    RobotContainer.m_shooterWheel.disable();
  }

  public static boolean isBallContained() {
    return RobotContainer.m_balldetecter.containsBall();
  }
  public static boolean isBallShot() {
    return RobotContainer.m_balldetecter.ballShot();
  }
}
