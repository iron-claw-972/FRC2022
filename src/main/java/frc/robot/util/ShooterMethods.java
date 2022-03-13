package frc.robot.util;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.controls.Operator;

public class ShooterMethods {

  // cargo arm methods
  public static void setAngle(double angle) {
    RobotContainer.m_cargoRotator.setPosition(angle);
  }

  public static double getOptimalShooterSpeed() {
    double distance = Units.metersToInches(RobotContainer.m_limelight.getHubDistance(RobotContainer.m_cargoRotator.currentAngle()));
    SmartDashboard.putNumber("Distance", distance);

    double speed = ShooterMethods.isArmBack() ? -(9.6 * distance + 1405) : -(8.53 * distance + 1304);
    //System.out.println("Speed: " + speed);
    //System.out.println("Distance: " + distance);
    return speed;
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
    // return RobotContainer.m_cargoRotator.isFrontOutakeFar() || RobotContainer.m_cargoRotator.isFrontOutakeNear();
    // return RobotContainer.m_cargoRotator.currentAngle() <= Operator.cargoConstants.kFrontOuttakeFarPos + 3 &&
    //   RobotContainer.m_cargoRotator.currentAngle() <= Operator.cargoConstants.kFrontOuttakeNearPos + 3;
    return RobotContainer.m_cargoRotator.currentAngle() < 133;
  }

  //

  // belt methods
  public static void setBeltSpeed(double speed) {
    RobotContainer.m_cargoBelt.setOutput(speed);
  }

  public static void setBeltPower(double power) {
    RobotContainer.m_cargoBelt.setPower(power);
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
    double velocity = SmartDashboard.getNumber("Test shooter velocity", 0);
    // double rpm = -(184*velocity - 1162);
    // double rpm = -((-0.5062*Math.pow(velocity, 4)) + (44.7613*Math.pow(velocity, 3)) + (-1460.94*Math.pow(velocity, 2)) + (21025.8*Math.pow(velocity, 1)) + (-110780));
    double rpm = -(178*velocity -1100);
    RobotContainer.m_cargoShooter.setSpeed(rpm);
  }
  public static void setIntakeSpeed(double speed) {
    RobotContainer.m_cargoShooter.setSpeed(speed);
  }
  
  public static void enableWheel() {
    RobotContainer.m_cargoShooter.enable();
  }
  public static void disableWheel() {
    RobotContainer.m_cargoShooter.disable();
  }

  public static boolean isWheelAtSetpoint() {
    boolean reachedSetpoint = RobotContainer.m_cargoShooter.reachedSetpoint();
    if (reachedSetpoint) {
      System.out.println("Reached shooter wheel setpoint");
    }
    return reachedSetpoint;
  }

  public static void stopWheel() {
    RobotContainer.m_cargoShooter.setStop();
  }
  //

  public static void enableAll() {
    RobotContainer.m_cargoRotator.enable();
    RobotContainer.m_cargoBelt.enable();
    RobotContainer.m_cargoShooter.enable();
  }

  public static void disableShiitake() {
    RobotContainer.m_cargoBelt.disable();
    RobotContainer.m_cargoShooter.disable();
  }

  public static boolean isBallContained() {
    // System.out.println(RobotContainer.m_balldetector.containsBallSecurely());
    return RobotContainer.m_balldetector.containsBallSecurely();
  }

  public static boolean isBallShot() {
    return !RobotContainer.m_balldetector.containsBall();
  }

  public static void multiSetter(double armAngle, double beltPower, double shooterSpeed) {
    RobotContainer.m_cargoRotator.setPosition(armAngle);
    RobotContainer.m_cargoBelt.setPower(beltPower);
    RobotContainer.m_cargoShooter.setSpeed(shooterSpeed);
  }

}
