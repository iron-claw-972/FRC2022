package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControllerFactory;
import frc.robot.util.LimitSwitch;
import frc.robot.controls.Operator;
import frc.robot.robotConstants.climbExtender.MarinusClimbExtenderConstants;


public class ClimbExtender extends SubsystemBase {
  MarinusClimbExtenderConstants constants = new MarinusClimbExtenderConstants();
  private boolean enabled = false;
  private boolean manualEnabled = false;
  private final WPI_TalonFX m_motor;
  private String side;

  public PIDController extenderPID = new PIDController(constants.kP, constants.kI, constants.kD);
  private LimitSwitch limitSwitch;
  
  private double setpoint;

  // it was requested to use multiple objects for the extender because one might fail
  public ClimbExtender(boolean isLeft) {
    // if the arm is left, the tick value is inverted && objects are assigned correctly
    if (isLeft) {
      m_motor = ControllerFactory.createTalonFX(constants.kLeftExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kNeutral); // initializes the motor
      side = "Left"; // the direction for shuffleboard's use
      m_motor.setInverted(true);
      limitSwitch = new LimitSwitch(constants.kExtLeftLimitSwitch, constants.kExtLimitSwitchDebouncer);
    }
    else {
      // otherwise, just assign the motor object to the right
      m_motor = ControllerFactory.createTalonFX(constants.kRightExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kNeutral); // initializes the motor
      side = "Right"; // the direction for shuffleboard's use
      limitSwitch = new LimitSwitch(constants.kExtRightLimitSwitch, constants.kExtLimitSwitchDebouncer);
    }

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    m_motor.configForwardSoftLimitThreshold(constants.kSoftLimit, 10);

    // so that the limiters are enabled
    // TODO: If the motors don't move, CHECK TO SEE IF THE LIMITER IS TOO LOW!
    m_motor.configForwardSoftLimitEnable(true, 10);

    // so that I in the PID doesn't accumulate
    extenderPID.reset();

    // set the PID's tolerance
    extenderPID.setTolerance(constants.kExtenderTolerance);
  }

  // check if the setpoint is reached
  public boolean reachedSetpoint() {
    // returns true if the limit switch is hit or if the extension is within tolerance of its setpoint
    return limitSwitch.risingEdge() || (currentExtensionRaw() < setpoint + constants.kExtenderTolerance && currentExtensionRaw() > setpoint - constants.kExtenderTolerance);
  }

  // use this method to prevent the PID from accumulating too much speed
  public void resetPID() {
    extenderPID.reset();
  }

  // use this ONLY to set the current position as zero
  public void zero() {
    m_motor.setSelectedSensorPosition(0.0);
  }

  // remove the soft limiter and have manual override
  public void removeLimiter() {
    m_motor.configForwardSoftLimitEnable(false, 0);
    manualEnabled = true;
  }

  // enable the soft limiter and go back to default controls
  public void enableLimiter() {
    m_motor.configForwardSoftLimitThreshold(constants.kSoftLimit, 10);
    m_motor.configForwardSoftLimitEnable(true, 0);
    manualEnabled = false;
  }

  // called in RobotContainer by button binds
  public void set(double distance) {
    setpoint = distance;
  }

  public void setOffset(double offset) {
    m_motor.setSelectedSensorPosition(m_motor.getSelectedSensorPosition() + offset);
  }

  // returns the current extension in ticks
  public double currentExtensionRaw() {
    return m_motor.getSelectedSensorPosition();
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

  // checks to see if the extender is fullycompressed
  public boolean compressionLimitSwitch() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {

    if (manualEnabled) {
    
    if (side.equals("Left")) {
      if (Operator.controller.getJoystickAxis().leftY() > 0.1) {
        setOutput(-0.2);
        enabled = false;
      } else if (Operator.controller.getJoystickAxis().leftY() < -0.1) {
        setOutput(0.2);
        enabled = false;
      } else {
        setOutput(0);
      }
    }

    if (side.equals("Right")) {
      if (Operator.controller.getJoystickAxis().rightY() > 0.1) {
        setOutput(-0.2);
        enabled = false;
      } else if (Operator.controller.getJoystickAxis().rightY() < -0.1) {
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