package frc.robot.util;


import frc.robot.RobotContainer;

public class ShooterMethods {
  // cargo arm methods
  public static void setAngle(double angle) {
    System.out.println("set angle");
    RobotContainer.m_cargoRotator.setPosition(angle);
  }

  public static void enableArm() {
    RobotContainer.m_cargoRotator.enable();
  }

  public static void disableArm() {
    RobotContainer.m_cargoRotator.disable();
  }

  public static boolean isArmAtSetpoint() {
    return RobotContainer.m_cargoRotator.reachedSetpoint();
  }

  public static boolean isArmBack(){
    return RobotContainer.m_cargoRotator.isBackOutakeFar() || RobotContainer.m_cargoRotator.isBackOutakeNear();
  }

  public static boolean isArmFront(){
    return RobotContainer.m_cargoRotator.isFrontOutakeFar() || RobotContainer.m_cargoRotator.isFrontOutakeNear();
  }

  //

  // belt methods
  public static void setBeltSpeed(double speed) {
    RobotContainer.m_cargoBelt.setOutput(speed);
  }

  public static void enableBelt() {
    RobotContainer.m_cargoBelt.enable();
  }

  public static void disableBelt() {
    RobotContainer.m_cargoBelt.disable();
  }

  public static void stopBelt() {
    RobotContainer.m_cargoShooter.setStop();
  }
  //

  // wheel methods
  public static void setWheelSpeed(double speed) {
    RobotContainer.m_cargoShooter.setSpeed(speed);
  }

  public static void enableWheel() {
    RobotContainer.m_cargoShooter.enable();
  }
  public static void disableWheel() {
    RobotContainer.m_cargoShooter.disable();
  }

  public static boolean isWheelAtSetpoint() {
    return RobotContainer.m_cargoShooter.reachedSetpoint();
  }

  public static void stopWheel() {
    RobotContainer.m_cargoShooter.setStop();
  }
  //

  public static void enableAll() {
    System.out.println("enabled");
    RobotContainer.m_cargoRotator.enable();
    RobotContainer.m_cargoBelt.enable();
    RobotContainer.m_cargoShooter.enable();
  }

  public static void disableShooter() {
    System.out.println("interrupted");
    RobotContainer.m_cargoBelt.disable();
    RobotContainer.m_cargoShooter.disable();
  }

  public static boolean isBallContained() {
    System.out.println(RobotContainer.m_balldetector.containsBallSecurely());
    return RobotContainer.m_balldetector.containsBallSecurely();
  }

  public static boolean isBallShot() {
    return !isBallContained();
  }

  public static void multiSetter(double armAngle, double beltPower, double shooterSpeed) {
    System.out.println("setting");
    RobotContainer.m_cargoRotator.setPosition(armAngle);
    RobotContainer.m_cargoBelt.setPower(beltPower);
    RobotContainer.m_cargoShooter.setSpeed(shooterSpeed);
  }

}
