package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControllerFactory;
import frc.robot.controls.Operator;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;


public class ClimbExtender extends SubsystemBase {
  TraversoClimbExtenderConstants constants = new TraversoClimbExtenderConstants();
  private boolean enabled = false;
  private final WPI_TalonFX m_motor;
  private String side;
  private double offset = 0;

  public PIDController extenderPID = new PIDController(constants.kP, constants.kI, constants.kD);
  
  private double setpoint;

  // it was requested to use multiple objects for the extender because one might fail
  public ClimbExtender(boolean isLeft) {
    // if the arm is left, the tick value is inverted && objects are assigned correctly
    if (isLeft) {
      m_motor = ControllerFactory.createTalonFX(constants.kLeftExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast); // initializes the motor
      side = "Left"; // the direction for shuffleboard's use
      m_motor.setInverted(true);
    }
    else {
      // otherwise, just assign the motor object to the right
      m_motor = ControllerFactory.createTalonFX(constants.kRightExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast); // initializes the motor
      side = "Right"; // the direction for shuffleboard's use
    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    m_motor.configReverseSoftLimitThreshold(0, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    // TODO: Update this max forward limit!
    SmartDashboard.putNumber("Max Extension Ticks", constants.kExtenderMaxArmTicks);
    m_motor.configForwardSoftLimitThreshold(SmartDashboard.getNumber("Max Extension Ticks", constants.kExtenderMaxArmTicks), 10);

    // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
    m_motor.setSelectedSensorPosition(0.0);

    // so that the limiters are enabled
    // TODO: If the motors don't move, CHECK TO SEE IF THE LIMITER IS TOO LOW!
    m_motor.configForwardSoftLimitEnable(true, 10);
    m_motor.configReverseSoftLimitEnable(true, 10);

    extenderPID.reset();

    // set the PID's tolerance
    extenderPID.setTolerance(constants.kExtenderTolerance);
  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- 10), return true, otherwise return false
    //return extenderPID.atSetpoint();
    System.out.println(side + " " + currentExtensionRaw());
    return currentExtensionRaw() > setpoint + constants.kExtenderTolerance && currentExtensionRaw() < setpoint - constants.kExtenderTolerance;
  }

  public void resetPID() {
    extenderPID.reset();
  }

  public void zero() {
    offset = -currentExtension();
  }

  public void removeLimiter() {
    m_motor.configForwardSoftLimitEnable(false, 0);
  }

  public void enableLimiter() {
    m_motor.configForwardSoftLimitEnable(true, 0);
  }

  // called in RobotContainer by button binds
  public void set(double distance) {
    setpoint = distance;
  }

  // returns the current extension in inches
  public double currentExtension() {
    return m_motor.getSelectedSensorPosition() - offset;
  }

  // returns the current extension in ticks
  public double currentExtensionRaw() {
    return m_motor.getSelectedSensorPosition() - offset;
  }

  // enables the extender (wow!)
  public void enable() {
    enabled = true;
  }
  
  // disables the extender
  public void disable() {
    enabled = false;
    m_motor.set(0);
  }

  // tells the motor object to drive at a speed that the PID sets the motorPower to be
  public void setOutput(double motorPower) {
    m_motor.set(MathUtil.clamp(motorPower, constants.kMotorClampDown, constants.kMotorClampUp));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(side + " Extension", currentExtension());
    SmartDashboard.putNumber(side + " extension raw", currentExtensionRaw());

    if (Operator.controller.getJoystickAxis().leftY() > 0.1) {
      setOutput(-0.2);
      enabled = false;
    } else if (Operator.controller.getJoystickAxis().leftY() < -0.1) {
      setOutput(0.2);
      enabled = false;
    } else {
      setOutput(0);
    }

    if(enabled) {
      // motor power is set to the extender pid's calculation
      setOutput(extenderPID.calculate(currentExtensionRaw(), setpoint));
    }
  }

  public String getSide() {
    return side;
  }

  public boolean isEnabled(){
    return enabled;
  }
} 