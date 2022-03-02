package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoRotator extends SubsystemBase {
  private TraversoCargoRotatorConstants constants = new TraversoCargoRotatorConstants();

  private boolean enabled = false;
  private final DutyCycleEncoder encoder;
  private final WPI_TalonFX m_motor;

  private double setpoint = constants.kIntakePos;
  private double feedforward;
  private double outputVoltage;

  public PIDController cargoRotatorPID = new PIDController(constants.kP, constants.kI, constants.kD);

  public CargoRotator() {
    encoder = new DutyCycleEncoder(constants.kArmEncoder);
    m_motor = ControllerFactory.createTalonFX(constants.kArmMotor, constants.kSupplyCurrentLimit,
        constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast);

    // set the tolerance allowed for the PID
    cargoRotatorPID.setTolerance(constants.kArmTolerance);
  }

  @Override
  public void periodic() {
    feedforward = calculateFeedForward(setpoint);
    // System.out.println("Setpoint: " + setpoint);
    outputVoltage = -(cargoRotatorPID.calculate(currentAngle(), setpoint) + feedforward);
    if (enabled) {
      setVoltage(outputVoltage);
    }
  }

  public void resetPID() {
    cargoRotatorPID.reset();
  }

  public double calculateFeedForward(double setpoint){
    // adjusts for the center of mass. only does feedforward if its not on the hardstop
    if (currentAngle() > (constants.kIntakePos + constants.kFeedForwardHardstopTolerance) && 
        currentAngle() < (constants.kStowPos - constants.kFeedForwardHardstopTolerance)){
      return cosineOfAngle(setpoint - constants.kFeedForwardOffsetAngle) * constants.kFeedForward;
    }
    else{
      return 0;
    }
  }

  public double currentAngleRaw() {
    //fixes position of encoder when starting on wrong side of 0 position
    if (encoder.get() > 0.5) {
      return encoder.get() - 1.0;
    } else {
      return encoder.get();
    }
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    double angle = (currentAngleRaw() + constants.kOffset) * constants.kArmDegreeMultiple;
    if (angle < -60) {
      angle += 360;
    }
    if (angle > 200) {
      angle -= 360;
    }
    return angle;
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
    return cargoRotatorPID.atSetpoint();
  }

  // enables PID
  public void enable() {
    enabled = true;
    resetPID();
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

  public double getSetpoint() {
    return setpoint;
  }

  public double cosineOfAngle(double angle) {
    return Math.cos(angle * (Math.PI / 180.0));
  }

  public boolean isIntake() {
    return (constants.kIntakePos == setpoint);
  }

  public boolean isStow() {
    return (constants.kStowPos == setpoint);
  }

  public boolean isFrontOutakeNear() {
    return (constants.kFrontOuttakeNearPos == setpoint);
  }

  public boolean isFrontOutakeFar() {
    return (constants.kFrontOuttakeFarPos == setpoint);
  }

  public boolean isFront(){
    return isFrontOutakeFar() || isFrontOutakeNear();
  }

  public boolean isBackOutakeNear() {
    return (constants.kBackOuttakeNearPos == setpoint);
  }

  public boolean isBackOutakeFar() {
    return (constants.kBackOuttakeFarPos == setpoint);
  }
  public boolean isEnabled() {
    return enabled;
  }

}