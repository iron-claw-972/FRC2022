package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import ctre_shims.TalonEncoderSim;
import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private boolean m_enabled = false;
  private final DutyCycleEncoder m_encoder;
  private final WPI_TalonFX m_motor;

  private double m_setpoint = Constants.arm.kIntakePos;
  private double m_feedforward;
  private double m_outputVoltage;

  private SingleJointedArmSim m_simArm;
  private DutyCycleEncoderSim m_encoderSim;

  public PIDController m_armPID = new PIDController(Constants.arm.kP, Constants.arm.kI, Constants.arm.kD);

  public Arm() {
    this(
      new DutyCycleEncoder(Constants.arm.kArmEncoder),
      ControllerFactory.createTalonFX(Constants.arm.kArmMotor, Constants.arm.kSupplyCurrentLimit,
        Constants.arm.kSupplyTriggerThreshold, Constants.arm.kSupplyTriggerDuration, Constants.arm.kNeutral)
    );
  }

  public Arm(DutyCycleEncoder encoder, WPI_TalonFX motor) {
    m_encoder = encoder;
    m_motor = motor;

    // set the tolerance allowed for the PID
    m_armPID.setTolerance(Constants.arm.kArmTolerance);

    if (RobotBase.isSimulation()) {
      m_simArm = new SingleJointedArmSim(
        Constants.arm.kGearbox, 
        Constants.arm.kGearRatio, 
        Constants.arm.kMomentOfInertia, 
        Constants.arm.kLength, 
        Constants.arm.kMinAngleRads, 
        Constants.arm.kMaxAngleRads, 
        Constants.arm.kMassKg, 
        true
      );

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_encoderSim = new DutyCycleEncoderSim(m_encoder);
    }
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      m_feedforward = calculateFeedForward(m_setpoint);
      m_outputVoltage = -(m_armPID.calculate(currentAngle(), m_setpoint) + m_feedforward);
      setVoltage(m_outputVoltage);
    }
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_simArm.setInput(m_motor.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    m_simArm.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(Units.radiansToDegrees(m_simArm.getAngleRads()));
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
    return m_encoder.get();
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    double angle = (currentAngleRaw() + Constants.arm.kOffset) * Constants.arm.kArmDegreeMultiple;
    return MathUtil.clamp(angle, 0, 175);
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