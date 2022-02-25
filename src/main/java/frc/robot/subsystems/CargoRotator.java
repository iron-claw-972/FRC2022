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

  private boolean enabled = true;
  private final DutyCycleEncoder encoder;
  private final WPI_TalonFX m_motor;

  private double setpoint = constants.kIntakePos;
  private double feedforward;
  private double outputVoltage;

  private PIDController armPID = new PIDController(constants.kP, constants.kI, constants.kD);

  public CargoRotator() {
    enable();
    encoder = new DutyCycleEncoder(constants.kArmEncoder);
    m_motor = ControllerFactory.createTalonFX(constants.kArmMotor, constants.kSupplyCurrentLimit,
        constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast);

    // set the tolerance allowed for the PID
    armPID.setTolerance(constants.kArmTolerance);
    SmartDashboard.putNumber("cargo rotator setpoint", constants.kIntakePos);
    SmartDashboard.putData("Cargo Rotator PID", armPID);
  }

  @Override
  public void periodic() {
    setpoint = SmartDashboard.getNumber("cargo rotator setpoint", 2);
    feedforward = calculateFeedForward(setpoint);
    outputVoltage = -(armPID.calculate(currentAngle(), setpoint) + feedforward);
    SmartDashboard.putNumber("voltage", outputVoltage);
    if (enabled) {
      setVoltage(outputVoltage);
    }
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
    return (currentAngleRaw() + constants.kOffset) * constants.kArmDegreeMultiple;
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
    System.out.println("called set position to angle: " + angle);
    setpoint = angle;
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

  public void loadCargoRotatorShuffleboard() {
    // a pop-up in shuffleboard that allows you to see how much the arm extended in
    // inches
    SmartDashboard.putNumber("Cargo Arm Angle", currentAngle());
    // shows if the rotator is enabled/disabled
    SmartDashboard.putBoolean("Cargo Rotator", enabled);
    SmartDashboard.putNumber("Raw Angle", currentAngleRaw());
    setpoint = SmartDashboard.getNumber("cargo rotator setpoint", 0);
  }
}