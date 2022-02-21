package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoRotator extends SubsystemBase {
  private TraversoCargoRotatorConstants constants = new TraversoCargoRotatorConstants();

  private boolean enabled = false;
  private final DutyCycleEncoder encoder;
  private final WPI_TalonFX m_motor;

  private double setPoint = 90;
  private double encoderOffset;

  private PIDController armPID = new PIDController(constants.kP , constants.kI , constants.kD);
  ArmFeedforward feedforward = new ArmFeedforward(constants.kS, constants.kG, constants.kV, constants.kA);
  
  public CargoRotator() {
    encoder = new DutyCycleEncoder(constants.kArmEncoder);
    m_motor = ControllerFactory.createTalonFX(constants.kArmMotor, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast);
    encoderOffset = constants.kArmEncoderOffset;
    
    // set the tolerance allowed for the PID
    armPID.setTolerance(constants.kArmTolerance);
    setEncoder(80);
  }

  @Override
  public void periodic() {
    if(enabled) {
      // set the arm power according to a PID
      // setOutput(armPID.calculate(currentAngle(), setPoint));
      setVoltage(armPID.calculate(currentAngle(), setPoint) + feedforward.calculate(setPoint*(Math.PI/180), 0));
    }
  }

  public double currentAngleRaw() {
    return encoder.get();
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    return encoder.get() * constants.kArmDegreeMultiple + encoderOffset;
  }

  // 80 is all the way forward and  125 is all the way back
  public void setEncoder(double angle) { 
    encoderOffset = angle - encoder.get() * constants.kArmDegreeMultiple;
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
    return armPID.atSetpoint();
  }

  //enables PID
  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
    // if the subsystem is disabled, do not spin the motor
    m_motor.set(0);
  }

  public void setOutput(double motorPower){
    m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  public void setVoltage(double motorPower){
    m_motor.setVoltage(MathUtil.clamp(motorPower, -constants.kMotorClamp*12, constants.kMotorClamp*12));
  }

  // sets PID Goal
  public void setPosition(double angle){
    setPoint = angle;
  }

/*
  public boolean isIntake(){
    return (constants.kIntakePos - constants.kArmTolerance < currentAngle() &&
        currentAngle() < constants.kIntakePos + constants.kArmTolerance);
  }
  
  public boolean isStow(){
    return (constants.kStowPos - constants.kArmTolerance < currentAngle() &&
        currentAngle() < constants.kStowPos + constants.kArmTolerance);
  }

  public boolean isFrontOutakeNear(){
    return (constants.kFrontOutakeNearPos - constants.kArmTolerance < currentAngle() &&
        currentAngle() < constants.kFrontOutakeNearPos + constants.kArmTolerance);
  }

  public boolean isFrontOutakeFar(){
    return (constants.kFrontOutakeFarPos - constants.kArmTolerance < currentAngle() &&
        currentAngle() < constants.kFrontOutakeFarPos + constants.kArmTolerance);
  }

  public boolean isBackOutakeNear(){
    return (constants.kBackOutakeNearPos - constants.kArmTolerance < currentAngle() &&
        currentAngle() < constants.kBackOutakeNearPos + constants.kArmTolerance);
  }

  public boolean isBackOutakeFar(){
    return (constants.kBackOutakeFarPos - constants.kArmTolerance < currentAngle() &&
        currentAngle() < constants.kBackOutakeFarPos + constants.kArmTolerance);
  }
*/

  public boolean isIntake(){
    return (constants.kIntakePos == setPoint);
  }

  public boolean isStow(){
      return (constants.kStowPos == setPoint);
  }

  public boolean isFrontOutakeNear(){
    return (constants.kFrontOutakeNearPos == setPoint);
  }

  public boolean isFrontOutakeFar(){
    return (constants.kFrontOutakeFarPos == setPoint);
  }

  public boolean isBackOutakeNear(){
    return (constants.kBackOutakeNearPos == setPoint);
  }

  public boolean isBackOutakeFar(){
    return (constants.kBackOutakeFarPos == setPoint);
  }

  public void loadCargoRotatorShuffleboard() {
    // puts the PID into shuffleboard
    SmartDashboard.putData(armPID);
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Cargo Arm Angle", currentAngle());
    // shows if the rotator is enabled/disabled
    SmartDashboard.putBoolean("Cargo Rotator", enabled);
    // for zeroing the angle
    SmartDashboard.putNumber("Zero CargoR", 80);
  }
}