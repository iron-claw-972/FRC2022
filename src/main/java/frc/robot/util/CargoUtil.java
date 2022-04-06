package frc.robot.util;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.cargo.GetDistance;
import frc.robot.constants.Constants;

public class CargoUtil {
  // cargo arm methods
  public static void setAngle(double angle) {
    Robot.arm.resetPID();
    Robot.arm.setPosition(angle);
  }

  public static void setAngleOptimal() {
    setAngle(GetDistance.optimalStipeAngle);
  }

  public static void setAngle(DoubleSupplier angle) {
    setAngle(angle.getAsDouble());
  }

  public static double getWheelVelocity() {
    return Robot.shooter.getVelocity();
  }

  public static double getTargetHeightOffset(double physicalShooterAngle) {
    double physicalShooterAngleRad = Units.degreesToRadians(physicalShooterAngle);
    double targetHeightOffset = Constants.ll.kHubHeight // Height of hub
                              - Constants.ll.kPivotHeight // Height of stipe pivot
                              - (Constants.arm.kPivotToShootingExitPointLength * Math.sin(physicalShooterAngleRad)); // Height from pivot to shooter exit point
    return targetHeightOffset;
  }

  public static double getShootingDistance(double pivotDistance, double physicalShooterAngle) {
    double physicalShooterAngleRad = Units.degreesToRadians(physicalShooterAngle);

    // Horizontal distance from shooter exit point to center of hub
    double shootingDistance = pivotDistance // Distance from vision tape to pivot
                            + (Constants.ll.kHubDiameter / 2) // Radius of the hub
                            - (Constants.arm.kPivotToShootingExitPointLength * Math.cos(physicalShooterAngleRad)); // Subtract horizontal distance from stipe pivot to exit point of shooter (the midpoint between the centers of the two shooter wheels)
    return shootingDistance;
  }

  public static double limelightDistanceToPivotDistance(double limelightDistance, double limelightPosAngle) {
    double limelightPosAngleRad = Units.degreesToRadians(limelightPosAngle);
    double pivotDistance;
    if (limelightPosAngle - Constants.ll.kStipeToLimelightPosAngularOffset + Constants.ll.kStipeToLimelightFaceAngularOffset < 90) {
      // Front
      pivotDistance = limelightDistance
                            + (Constants.ll.kPivotToLimelightLength * Math.cos(limelightPosAngleRad)); // Horizontal distance from limelight to stipe pivot
    } else {
      // Back
      pivotDistance = limelightDistance
                            - (Constants.ll.kPivotToLimelightLength * Math.cos(limelightPosAngleRad)); // Horizontal distance from limelight to stipe pivot
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
    return Robot.arm.currentAngle();
  }

  public static void enableArm() {
    Robot.arm.enable();
  }

  public static void disableArm() {
    Robot.arm.disable();
  }

  public static boolean isArmAtSetpoint() {
    return Robot.arm.reachedSetpoint();
  }

  public static boolean isArmBack(){
    return !isArmFront();
  }

  public static boolean isArmFront(){
    // return Robot.mArm.isFrontOutakeFar() || Robot.mArm.isFrontOutakeNear();
    // return Constants.m_cargoRotator.currentAngle() <= Operator..arm.kFrontOuttakeFarPos + 3 &&
    //   Constants.m_cargoRotator.currentAngle() <= Operator..arm.kFrontOuttakeNearPos + 3;
    return Robot.arm.currentAngle() + Constants.arm.kStipeToPhysicalShooterAngularOffset < 90;
  }

  public static boolean isLimelightFaceFront() {
    return Robot.arm.currentAngle() + Constants.ll.kStipeToLimelightFaceAngularOffset < 90;
  }

  // public static double getFrontStaticShootingSpeed() {
  //   return SmartDashboard.getNumber("Front Shooting velocity", 0);
  // }

  // public static double getFrontStaticStipeAngle() {
  //   return SmartDashboard.getNumber("Front Stipe angle", 0);
  // }

  // public static double getBackStaticShootingSpeed() {
  //   return SmartDashboard.getNumber("Back Shooting velocity", 0);
  // }

  // public static double getBackStaticStipeAngle() {
  //   return SmartDashboard.getNumber("Back Stipe angle", 0);
  // }

  //

  // belt methods
  public static void setBeltSpeed(double speed) {
    Robot.belt.setOutput(speed);
  }

  public static void setBeltPower(double power) {
    Robot.belt.setPower(power);
  }

  public static void enableBelt() {
    Robot.belt.enable();
  }

  public static void disableBelt() {
    Robot.belt.disable();
  }

  public static void stopBelt() {
    Robot.shooter.setStop();
  }

  public static double velocityToRPM(DoubleSupplier speed, boolean isFront) {
    double velocity = speed.getAsDouble();
    double rpm;
    if (isFront) {
      // rpm = -(178*velocity - 1100);
      // rpm = -(122*velocity - 1167);
      rpm = -(126*velocity - 1264);
    } else {
      rpm = -(372*velocity - 5943);
      // rpm = -(294*velocity - 3628);
    }
    return rpm;
  }

  // wheel methods
  public static void setWheelSpeed(DoubleSupplier speed, boolean isFront) {
    Robot.shooter.setSpeed(velocityToRPM(speed, isFront));
  }
  public static void setWheelRPM(double speed) {
    Robot.shooter.setSpeed(speed);
  }

  public static void setWheelRPM(DoubleSupplier speed) {
    setWheelRPM(speed.getAsDouble());
  }

  public static double getTestShooterSpeed() {
    return SmartDashboard.getNumber("Test shooter speed", 0);
  }

  public static double getTestArmAngle() {
    return SmartDashboard.getNumber("Test arm angle", 0);
  }
  
  public static void enableWheel() {
    Robot.shooter.enable();
  }
  public static void disableWheel() {
    Robot.shooter.disable();
  }

  public static boolean isWheelAtSetpoint() {
    boolean reachedSetpoint = Robot.shooter.reachedSetpoint();
    if (reachedSetpoint) {
      System.out.println("Reached shooter wheel setpoint");
    }
    return reachedSetpoint;
  }

  // public static boolean isRedAlliance() {
  //   return SmartDashboard.getBoolean("Is Red Alliance", Constants.kIsRedAlliance);
  // }

  public static void stopWheel() {
    Robot.shooter.setStop();
  }
  //

  public static void enableAll() {
    Robot.arm.enable();
    Robot.belt.enable();
    Robot.shooter.enable();
  }

  public static void disableShiitake() {
    Robot.belt.disable();
    Robot.shooter.disable();
  }

  public static boolean isBallContainedSecurely() {
    // System.out.println(Robot.m_balldetector.containsBallSecurely());
    return Robot.ballDetection.containsBallSecurely();
  }

  public static double getHubDistance(double stipeAngle) {
    return Robot.ll.getHubDistance(stipeAngle);
  }

  public static void setUpperHubPipeline() {
    Robot.ll.setUpperHubPipeline();
  }

  public static void setBallPipeline(boolean isRedBall) {
    Robot.ll.setBallPipeline(isRedBall);
  }

  public static boolean isBallContained() {
    // System.out.println(Robot.m_balldetector.containsBallSecurely());
    return Robot.ballDetection.containsBall();
  }

  public static boolean isBallShot() {
    return !Robot.ballDetection.containsBall();
  }

  public static void multiSetter(double armAngle, double beltPower, double shooterSpeed) {
    Robot.arm.setPosition(armAngle);
    Robot.belt.setPower(beltPower);
    Robot.shooter.setSpeed(shooterSpeed);
  }

}
