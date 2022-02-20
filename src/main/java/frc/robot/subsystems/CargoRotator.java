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
    m_motor = ControllerFactory.createTalonFX(constants.kArmMotor, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, false);
    encoderOffset = constants.kArmEncoderOffset;
    
    // set the tolerance allowed for the PID
    armPID.setTolerance(constants.kArmTolerance);
    //Puts PID values on shuffle board for tuning the PID (to be commented out later)
    SmartDashboard.putNumber("P", constants.kP);
    SmartDashboard.putNumber("I", constants.kI);
    SmartDashboard.putNumber("D", constants.kD);
    SmartDashboard.putNumber("set encoder", 80);
    SmartDashboard.putNumber("goal", 90);
    setEncoder(80);
  }

  @Override
  public void periodic() {
    if(enabled) {

      // gets PID values from shuffle board for tuning the PID (to be commented out later)
      armPID.setP(SmartDashboard.getNumber("P", constants.kP));
      armPID.setI(SmartDashboard.getNumber("I", constants.kI));
      armPID.setD(SmartDashboard.getNumber("D", constants.kD));
      // setpoint = SmartDashboard.getNumber("goal", 0);

      // set the arm power according to a PID
      // setOutput(armPID.calculate(currentAngle(), setPoint));
      setVoltage(armPID.calculate(currentAngle(), setPoint) + feedforward.calculate(setPoint*(Math.PI/180), 0));
      SmartDashboard.putNumber("Cargo Arm Angle", currentAngle());
    }

    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Angle (Degrees)", currentAngle());
    // System.out.println(currentAngle());

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
    encoderOffset = angle // constants.kArmDegreeMultiple
          - encoder.get() * constants.kArmDegreeMultiple;
    System.out.println("set encoder");

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
  public void setGoal(double goal){
    setPoint = goal;
  }
}