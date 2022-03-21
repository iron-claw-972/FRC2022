package frc.robot.util;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.cargoCommands.GetDistance;

public class ShooterMethods {

  // cargo arm methods
  public static void setAngle(double angle) {
    RobotContainer.m_cargoRotator.resetPID();
    RobotContainer.m_cargoRotator.setPosition(angle);
  }

  public static void setAngleOptimal() {
    setAngle(GetDistance.optimalStipeAngle);
  }

  public static void setAngle(DoubleSupplier angle) {
    setAngle(angle.getAsDouble());
  }

  public static double getWheelVelocity() {
    return RobotContainer.m_cargoShooter.getVelocity();
  }

  public static double getTargetHeightOffset(double physicalShooterAngle) {
    double physicalShooterAngleRad = Units.degreesToRadians(physicalShooterAngle);
    double targetHeightOffset = RobotContainer.limelightConstants.kHubHeight // Height of hub
                              - RobotContainer.limelightConstants.kPivotHeight // Height of stipe pivot
                              - (RobotContainer.cargoConstants.kPivotToShootingExitPointLength * Math.sin(physicalShooterAngleRad)); // Height from pivot to shooter exit point
    return targetHeightOffset;
  }

  public static double getShootingDistance(double pivotDistance, double physicalShooterAngle) {
    double physicalShooterAngleRad = Units.degreesToRadians(physicalShooterAngle);

    // Horizontal distance from shooter exit point to center of hub
    double shootingDistance = pivotDistance // Distance from vision tape to pivot
                            + (RobotContainer.limelightConstants.kHubDiameter / 2) // Radius of the hub
                            - (RobotContainer.cargoConstants.kPivotToShootingExitPointLength * Math.cos(physicalShooterAngleRad)); // Subtract horizontal distance from stipe pivot to exit point of shooter (the midpoint between the centers of the two shooter wheels)
    return shootingDistance;
  }

  public static double limelightDistanceToPivotDistance(double limelightDistance, double limelightPosAngle) {
    double limelightPosAngleRad = Units.degreesToRadians(limelightPosAngle);
    double pivotDistance;
    if (limelightPosAngle - RobotContainer.limelightConstants.kStipeToLimelightPosAngularOffset + RobotContainer.limelightConstants.kStipeToLimelightFaceAngularOffset < 90) {
      // Front
      pivotDistance = limelightDistance
                            + (RobotContainer.limelightConstants.kPivotToLimelightLength * Math.cos(limelightPosAngleRad)); // Horizontal distance from limelight to stipe pivot
    } else {
      // Back
      pivotDistance = limelightDistance
                            - (RobotContainer.limelightConstants.kPivotToLimelightLength * Math.cos(limelightPosAngleRad)); // Horizontal distance from limelight to stipe pivot
    }
    return pivotDistance;
  }

  public static double getOptimalShooterSpeed(double shootingAngle, double targetHeightOffset, double shooterDistance) {
    double shootingAngleRad = Units.degreesToRadians(shootingAngle);
    double optimalSpeed = (shooterDistance / Math.cos(shootingAngleRad))
                        * Math.sqrt(Constants.GRAVITATIONAL_ACCEL
                          / (2 * (shooterDistance * Math.tan(shootingAngleRad) - targetHeightOffset)));
    return optimalSpeed;
  }

  public static double getOptimalShootingAngle(double sAngle, double shooterDistance, double targetHeightOffset) {
    double sAngleRad = Units.degreesToRadians(sAngle);
    double optimalAngle = Math.atan(
      (2 * targetHeightOffset / shooterDistance) - Math.tan(sAngleRad)
    );
    optimalAngle = Units.radiansToDegrees(optimalAngle);
    return optimalAngle;
  }

  public static double getArmAngle() {
    return RobotContainer.m_cargoRotator.currentAngle();
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
    return !isArmFront();
  }

  public static boolean isArmFront(){
    // return RobotContainer.m_cargoRotator.isFrontOutakeFar() || RobotContainer.m_cargoRotator.isFrontOutakeNear();
    // return RobotContainer.m_cargoRotator.currentAngle() <= Operator.cargoConstants.kFrontOuttakeFarPos + 3 &&
    //   RobotContainer.m_cargoRotator.currentAngle() <= Operator.cargoConstants.kFrontOuttakeNearPos + 3;
    return RobotContainer.m_cargoRotator.currentAngle() + RobotContainer.cargoConstants.kStipeToPhysicalShooterAngularOffset < 90;
  }

  public static boolean isLimelightFaceFront() {
    return RobotContainer.m_cargoRotator.currentAngle() + RobotContainer.limelightConstants.kStipeToLimelightFaceAngularOffset < 90;
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

  public static double velocityToRPM(DoubleSupplier speed, boolean isFront) {
    double velocity = speed.getAsDouble();
    double rpm;
    if (isFront) {
      rpm = -(178*velocity - 1100);
    } else {
      // rpm = -(372*velocity - 5943);
      rpm = -(294*velocity - 3628);
    }
    return rpm;
  }

  // wheel methods
  public static void setWheelSpeed(DoubleSupplier speed, boolean isFront) {
    RobotContainer.m_cargoShooter.setSpeed(velocityToRPM(speed, isFront));
  }
  public static void setWheelRPM(double speed) {
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

  public static boolean isBallContainedSecurely() {
    // System.out.println(RobotContainer.m_balldetector.containsBallSecurely());
    return RobotContainer.m_ballDetection.containsBallSecurely();
  }

  public static boolean isBallContained() {
    // System.out.println(RobotContainer.m_balldetector.containsBallSecurely());
    return RobotContainer.m_ballDetection.containsBall();
  }

  public static boolean isBallShot() {
    return !RobotContainer.m_ballDetection.containsBall();
  }

  public static void multiSetter(double armAngle, double beltPower, double shooterSpeed) {
    RobotContainer.m_cargoRotator.setPosition(armAngle);
    RobotContainer.m_cargoBelt.setPower(beltPower);
    RobotContainer.m_cargoShooter.setSpeed(shooterSpeed);
  }

}
