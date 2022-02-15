package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.controls.Operator;
import frc.robot.robotConstants.cargoArm.TraversoCargoArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoArm extends SubsystemBase {
  TraversoCargoArmConstants constants;

  private boolean enabled = false;
  private final DutyCycleEncoder dce;
  private final WPI_TalonFX m_motor;

  private double setpoint = 0;
  private double encoderOffset = 0;

  private PIDController armPID = new PIDController(0.02, 0.0000, 0.0000);
  
  public CargoArm() {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    dce = new DutyCycleEncoder(constants.kArmEncoder);
    m_motor = ControllerFactory.createTalonFX(constants.kArmMotor);
    encoderOffset = constants.kArmEncoderOffset;
    armPID.setTolerance(constants.kArmTolerance);
    // SmartDashboard.putNumber("P", 0.02);
    // SmartDashboard.putNumber("I", 0.000);
    // SmartDashboard.putNumber("D", 0.000);
    // SmartDashboard.putNumber("set encoder", 0);
    // SmartDashboard.putNumber("goal", 0);
    // setEncoder(-45);
  }

  public double currentAngleRaw() {
    return dce.get();
  }

  // returns the current angle of the duty cycle encoder
  public double currentAngle() {
    return dce.get() * constants.kArmDegreeMultiple + encoderOffset;
  }

  // 80 is all the way forward and  125 is all the way back
  public void setEncoder(double angle) { 
    encoderOffset = angle / constants.kArmDegreeMultiple
          - dce.get() * constants.kArmDegreeMultiple;
    // System.out.println("set encoder");

  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- tolerance), return true, otherwise return false
    return armPID.atSetpoint();
  }

  // called in RobotContainer by button binds
  public void set(double distance) {
    setpoint = distance;
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
    m_motor.set(0);
    // if the subsystem is disabled, do not spin the motor
  }

  public void setOutput(double motorPower){
    m_motor.set(ControlMode.PercentOutput, -MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  @Override
  public void periodic() {
    if(enabled) {
      // armPID.setP(SmartDashboard.getNumber("P", 0.02));
      // armPID.setI(SmartDashboard.getNumber("I", 0.000));
      // armPID.setD(SmartDashboard.getNumber("D", 0.000));
      // setpoint = SmartDashboard.getNumber("goal", 0);
      // set the arm power according to a PID
      setOutput(armPID.calculate(currentAngle(), setpoint));
    }

    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Angle (Degrees)", currentAngle());
    // System.out.println(currentAngle());

  }
  
  public void setGoal(double goal){
    setpoint = goal;
  }
}