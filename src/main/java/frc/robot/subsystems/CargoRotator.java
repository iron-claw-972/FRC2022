package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoRotator extends SubsystemBase {
  private TraversoCargoRotatorConstants constants = new TraversoCargoRotatorConstants();

  private boolean enabled = false;
  private final DutyCycleEncoder encoder;
  private final WPI_TalonFX m_motor;

  private double setpoint = 0;
  private double feedForward = 0;

  private PIDController armPID = new PIDController(constants.kP, constants.kI, constants.kD);

  public CargoRotator() {
    encoder = new DutyCycleEncoder(constants.kArmEncoder);
    m_motor = ControllerFactory.createTalonFX(constants.kArmMotor, constants.kSupplyCurrentLimit,
        constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast);

    // set the tolerance allowed for the PID
    armPID.setTolerance(constants.kArmTolerance);
    SmartDashboard.putNumber("cargo rotator setpoint", 0);
    SmartDashboard.putNumber("cargo feed forward", 0);
    SmartDashboard.putData("Cargo Rotator PID", armPID);
  }

  @Override
  public void periodic() {
    enable();
    double ff = cosineOfAngle(setpoint - 30.0) * feedForward * ((currentAngle() < 5.0) ? 0.0 : 1.0);
    System.out.println("cos: " + cosineOfAngle(setpoint - 30.0));
    System.out.println("ff: " + feedForward);
    double yeehaw = -(armPID.calculate(currentAngle(), setpoint) + ff);
    SmartDashboard.putNumber("voltage", yeehaw);
    if (enabled) {
      setVoltage(yeehaw);
    }
  }

  public double currentAngleRaw() {
    if (encoder.get() > 0.5) {
      return encoder.get() - 1.0;
    } else {
      return encoder.get();
    }
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    return (currentAngleRaw() * constants.kArmDegreeMultiple) + (constants.kOffset * constants.kArmDegreeMultiple);
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
    return armPID.atSetpoint();
  }

  // enables PID
  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
    // if the subsystem is disabled, do not spin the motor
    m_motor.set(0);
  }

  // public void setOutput(double motorPower){
  // m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower,
  // -constants.kMotorClamp, constants.kMotorClamp));
  // }

  public void setVoltage(double motorPower) {
    m_motor.setVoltage(MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  // sets PID Goal
  public void setPosition(double angle) {
    setpoint = angle;
  }

  public double cosineOfAngle(double angle) {
    return Math.cos(angle * (Math.PI / 180.0));
  }

  /*
   * public boolean isIntake(){
   * return (constants.kIntakePos - constants.kArmTolerance < currentAngle() &&
   * currentAngle() < constants.kIntakePos + constants.kArmTolerance);
   * }
   * 
   * public boolean isStow(){
   * return (constants.kStowPos - constants.kArmTolerance < currentAngle() &&
   * currentAngle() < constants.kStowPos + constants.kArmTolerance);
   * }
   * 
   * public boolean isFrontOutakeNear(){
   * return (constants.kFrontOutakeNearPos - constants.kArmTolerance <
   * currentAngle() &&
   * currentAngle() < constants.kFrontOutakeNearPos + constants.kArmTolerance);
   * }
   * 
   * public boolean isFrontOutakeFar(){
   * return (constants.kFrontOutakeFarPos - constants.kArmTolerance <
   * currentAngle() &&
   * currentAngle() < constants.kFrontOutakeFarPos + constants.kArmTolerance);
   * }
   * 
   * public boolean isBackOutakeNear(){
   * return (constants.kBackOutakeNearPos - constants.kArmTolerance <
   * currentAngle() &&
   * currentAngle() < constants.kBackOutakeNearPos + constants.kArmTolerance);
   * }
   * 
   * public boolean isBackOutakeFar(){
   * return (constants.kBackOutakeFarPos - constants.kArmTolerance <
   * currentAngle() &&
   * currentAngle() < constants.kBackOutakeFarPos + constants.kArmTolerance);
   * }
   */

  public boolean isIntake() {
    return (constants.kIntakePos == setpoint);
  }

  public boolean isStow() {
    return (constants.kStowPos == setpoint);
  }

  public boolean isFrontOutakeNear() {
    return (constants.kFrontOutakeNearPos == setpoint);
  }

  public boolean isFrontOutakeFar() {
    return (constants.kFrontOutakeFarPos == setpoint);
  }

  public boolean isBackOutakeNear() {
    return (constants.kBackOutakeNearPos == setpoint);
  }

  public boolean isBackOutakeFar() {
    return (constants.kBackOutakeFarPos == setpoint);
  }

  public void loadCargoRotatorShuffleboard() {
    // a pop-up in shuffleboard that allows you to see how much the arm extended in
    // inches
    SmartDashboard.putNumber("Cargo Arm Angle", currentAngle());
    // shows if the rotator is enabled/disabled
    SmartDashboard.putBoolean("Cargo Rotator", enabled);
    SmartDashboard.putNumber("Raw Angle", currentAngleRaw());
    setpoint = SmartDashboard.getNumber("cargo rotator setpoint", 0);
    feedForward = SmartDashboard.getNumber("cargo feed forward", 0);
  }
}