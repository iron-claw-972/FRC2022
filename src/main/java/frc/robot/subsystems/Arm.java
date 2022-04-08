package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private boolean m_enabled = false;
  private final DutyCycleEncoder m_encoder;
  private final WPI_TalonFX m_motor;

  private double m_setpoint = Constants.arm.kIntakePos;
  private double m_feedforward;
  private double m_outputVoltage;

  public PIDController m_armPID = new PIDController(Constants.arm.kP, Constants.arm.kI, Constants.arm.kD);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(0.5),
              6,
              new Color8Bit(Color.kYellow)));

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
      //display for simulator on SmartDashboard
      SmartDashboard.putData("Arm Sim", m_mech2d);
      m_armTower.setColor(new Color8Bit(Color.kBlue));
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
    // Ideally we would set this to Units.radiansToDegrees(m_armSim.getAngleRads()) to set the arm to the simulation
    // with m_arm sim being a SingleJointedArmSim class, configured using the constants in ArmConstants
    // However it doesn't seem accurate enough
    m_arm.setAngle(m_setpoint);
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
    if (RobotBase.isSimulation()) {
      return true;
    }
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