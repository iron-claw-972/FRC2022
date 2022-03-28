package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoArm extends SubsystemBase {
  private boolean m_enabled = false;
  private final DutyCycleEncoder m_encoder;
  private final WPI_TalonFX m_motor;

  private double m_setpoint = Constants.arm.kIntakePos;
  private double m_feedforward;
  private double m_outputVoltage;

  public PIDController m_armPID = new PIDController(Constants.arm.kP, Constants.arm.kI, Constants.arm.kD);

  public CargoArm() {
    m_encoder = new DutyCycleEncoder(Constants.arm.kArmEncoder);
    m_motor = ControllerFactory.createTalonFX(Constants.arm.kArmMotor, Constants.arm.kSupplyCurrentLimit,
        Constants.arm.kSupplyTriggerThreshold, Constants.arm.kSupplyTriggerDuration, Constants.arm.kNeutral);

    // set the tolerance allowed for the PID
    m_armPID.setTolerance(Constants.arm.kArmTolerance);
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      m_feedforward = calculateFeedForward(m_setpoint);
      m_outputVoltage = -(m_armPID.calculate(currentAngle(), m_setpoint) + m_feedforward);
      setVoltage(m_outputVoltage);
    }
  }

  public void resetPID() {
    m_armPID.reset();
  }

  public double calculateFeedForward(double setpoint){
    // adjusts for the center of mass. only does feedforward if its not on the hardstop
    if (currentAngle() > (Constants.arm.kIntakePos + Constants.arm.kFeedForwardHardstopTolerance) && 
        currentAngle() < (Constants.arm.kStowPos - Constants.arm.kFeedForwardHardstopTolerance)){
      return cosineOfAngle(setpoint - Constants.arm.kFeedForwardOffsetAngle) * Constants.arm.kFeedForward;
    }
    else{
      return 0;
    }
  }

  public double currentAngleRaw() {
    //fixes position of encoder when starting on wrong side of 0 position
    return m_encoder.get();
    // if (encoder.get() > 0.5) {
    //   return encoder.get() - 1.0;
    // } else {
    //   return encoder.get();
    // }
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    double angle = (currentAngleRaw() + Constants.arm.kOffset) * Constants.arm.kArmDegreeMultiple;
    // if (angle < -60) {
    //   angle += 360;
    // }
    // if (angle > 200) {
    //   // angle -= 360;
    //   angle = angle % 360;
    // }
    return MathUtil.clamp(angle, 0, 175);
    // return angle;
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
    return m_armPID.atSetpoint();
  }

  // enables PID
  public void enable() {
    m_enabled = true;
    resetPID();
  }

  public void disable() {
    m_enabled = false;
    // if the subsystem is disabled, do not spin the motor
    m_motor.set(0);
  }

  // public void setOutput(double motorPower){
  // m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower,
  // -Constants.arm.kMotorClamp, Constants.arm.kMotorClamp));
  // }

  public void setVoltage(double motorPower) {
    m_motor.setVoltage(MathUtil.clamp(motorPower, -Constants.arm.kMotorClamp, Constants.arm.kMotorClamp));
  }

  // sets PID Goal
  public void setPosition(double angle) {
    m_setpoint = MathUtil.clamp(angle, 0, 175);
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public double cosineOfAngle(double angle) {
    return Math.cos(angle * (Math.PI / 180.0));
  }

  public boolean isIntake() {
    return (Constants.arm.kIntakePos == m_setpoint);
  }

  public boolean isStow() {
    return (Constants.arm.kStowPos == m_setpoint);
  }

  public boolean isFrontOutakeNear() {
    return (Constants.arm.kFrontOuttakeNearPos == m_setpoint);
  }

  public boolean isFrontOutakeFar() {
    return (Constants.arm.kFrontOuttakeFarPos == m_setpoint);
  }

  public boolean isFront(){
    return isFrontOutakeFar() || isFrontOutakeNear();
  }

  public boolean isBackOutakeNear() {
    return (Constants.arm.kBackOuttakeNearPos == m_setpoint);
  }

  public boolean isBackOutakeFar() {
    return (Constants.arm.kBackOuttakeFarPos == m_setpoint);
  }
  public boolean isEnabled() {
    return m_enabled;
  }

}