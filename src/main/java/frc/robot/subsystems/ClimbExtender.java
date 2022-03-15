package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControllerFactory;
import frc.robot.util.LimitSwitch;
import frc.robot.controls.ClimbOperator;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;


public class ClimbExtender extends SubsystemBase {
  TraversoClimbExtenderConstants constants = new TraversoClimbExtenderConstants();
  private boolean enabled = false;
  private boolean manualEnabled = false;
  private final WPI_TalonFX m_motor;
  private String side;
  private double offset = 0;

  public PIDController extenderPID = new PIDController(constants.kP, constants.kI, constants.kD);
  private LimitSwitch limitSwitch;
  
  private double setpoint;

  // it was requested to use multiple objects for the extender because one might fail
  public ClimbExtender(boolean isLeft) {
    // if the arm is left, the tick value is inverted && objects are assigned correctly
    if (isLeft) {
      m_motor = ControllerFactory.createTalonFX(constants.kLeftExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast); // initializes the motor
      side = "Left"; // the direction for shuffleboard's use
      m_motor.setInverted(true);

      limitSwitch = new LimitSwitch(constants.kExtLeftLimitSwitch, constants.kExtLimitSwitchDebouncer);
    }
    else {
      // otherwise, just assign the motor object to the right
      m_motor = ControllerFactory.createTalonFX(constants.kRightExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast); // initializes the motor
      side = "Right"; // the direction for shuffleboard's use

      limitSwitch = new LimitSwitch(constants.kExtRightLimitSwitch, constants.kExtLimitSwitchDebouncer);
    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    m_motor.configReverseSoftLimitThreshold(1000, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    // TODO: Update this max forward limit!
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
    // System.out.println(side + " extension: " + currentExtensionRaw() + ", setpoint: " + setpoint);
    // System.out.println(currentExtensionRaw() < setpoint + constants.kExtenderTolerance);
    // System.out.println(currentExtensionRaw() > setpoint - constants.kExtenderTolerance);
    //possibly the issue is that they don't both reach the setpoint at the same time? no probably not

    return limitSwitch.risingEdge() || (currentExtensionRaw() < setpoint + constants.kExtenderTolerance && currentExtensionRaw() > setpoint - constants.kExtenderTolerance);
  }

  public void resetPID() {
    extenderPID.reset();
  }

  public void zero() {
    offset = -currentExtensionRaw();
  }

  public void changeOffset(double amount) {
    offset += amount;
  }

  public void setReverseLimit(double amount) {
    m_motor.configReverseSoftLimitThreshold(amount, 10);
  }

  public void setForwardLimit(double amount) {
    m_motor.configForwardSoftLimitThreshold(amount, 10);
  }

  public void removeLimiter() {
    m_motor.configForwardSoftLimitEnable(false, 0);
    m_motor.configReverseSoftLimitThreshold(10, 10);
    manualEnabled = true;
  }

  public void enableLimiter() {
    m_motor.configForwardSoftLimitThreshold(SmartDashboard.getNumber("Max Extension Ticks", constants.kExtenderMaxArmTicks), 10);
    m_motor.configForwardSoftLimitEnable(true, 0);
    m_motor.configReverseSoftLimitThreshold(SmartDashboard.getNumber("Min Extension Ticks", 1000), 10);
    manualEnabled = false;
  }

  // called in RobotContainer by button binds
  public void set(double distance) {
    setpoint = distance;
  }

  // returns the current extension in inches
  // public double currentExtension() {
  //   return m_motor.getSelectedSensorPosition() + offset;
  // }

  // returns the current extension in ticks
  public double currentExtensionRaw() {
    return m_motor.getSelectedSensorPosition() + offset;
  }

  public double getVelocity() {
    return m_motor.getSelectedSensorVelocity();
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

  public boolean compressionLimitSwitch() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {

    if (manualEnabled) {
    
    if (side.equals("Left")) {
      if (ClimbOperator.controller.getJoystickAxis().leftY() > 0.1) {
        setOutput(-0.2);
        enabled = false;
      } else if (ClimbOperator.controller.getJoystickAxis().leftY() < -0.1) {
        setOutput(0.2);
        enabled = false;
      } else {
        setOutput(0);
      }
    }

    if (side.equals("Right")) {
      if (ClimbOperator.controller.getJoystickAxis().rightY() > 0.1) {
        setOutput(-0.2);
        enabled = false;
      } else if (ClimbOperator.controller.getJoystickAxis().rightY() < -0.1) {
        setOutput(0.2);
        enabled = false;
      } else {
        setOutput(0);
      }
    }

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