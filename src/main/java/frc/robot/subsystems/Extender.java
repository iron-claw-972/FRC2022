package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControllerFactory;
import frc.robot.util.LimitSwitch;
import lib.controllers.GameController.Axis;
import frc.robot.constants.Constants;
import frc.robot.controls.Operator;


public class Extender extends SubsystemBase {
  private boolean m_enabled = false;
  private boolean m_manualEnabled = false;
  private WPI_TalonFX m_motor;
  private String m_side;

  public PIDController m_extenderPID = new PIDController(Constants.extender.kP, Constants.extender.kI, Constants.extender.kD);
  private LimitSwitch m_limitSwitch;
  
  private double m_setpoint;

  public Extender(boolean isLeft) {
    this(isLeft, ControllerFactory.createTalonFX((
        isLeft ? Constants.extender.kLeftExtenderPort : Constants.extender.kRightExtenderPort), 
        Constants.extender.kSupplyCurrentLimit, 
        Constants.extender.kSupplyTriggerThreshold, 
        Constants.extender.kSupplyTriggerDuration,
        Constants.extender.kNeutral
      ),
      new LimitSwitch((
        isLeft ? Constants.extender.kExtLeftLimitSwitch : Constants.extender.kExtRightLimitSwitch), 
        Constants.extender.kExtLimitSwitchDebouncer
      )
    );
  }

  // it was requested to use multiple objects for the extender because one might fail
  public Extender(boolean isLeft, WPI_TalonFX motor, LimitSwitch limitSwitch) {
    m_motor = motor;
    m_limitSwitch = limitSwitch;

    if (isLeft) {
      // if the arm is left, the tick value is inverted && objects are assigned correctly
      m_side = "Left"; // the direction for shuffleboard's use
      m_motor.setInverted(true);
    }
    else {
      // otherwise, just assign the motor object to the right
      m_side = "Right"; // the direction for shuffleboard's use
    }

    // the tick value can't exceed the soft limit, checked every 10 milliseconds
    m_motor.configForwardSoftLimitThreshold(Constants.extender.kSoftLimit, 10);

    // so that the limiters are enabled
    m_motor.configForwardSoftLimitEnable(true, 10);

    // so that I in the PID doesn't accumulate
    m_extenderPID.reset();

    // set the PID's tolerance
    m_extenderPID.setTolerance(Constants.extender.kExtenderTolerance);
  }

  // check if the setpoint is reached
  public boolean reachedSetpoint() {
    // returns true if the limit switch is hit or if the extension is within tolerance of its setpoint
    return m_limitSwitch.risingEdge() || (currentExtensionRaw() < m_setpoint + Constants.extender.kExtenderTolerance && currentExtensionRaw() > m_setpoint - Constants.extender.kExtenderTolerance);
  }

  // use this method to prevent the PID from accumulating too much speed
  public void resetPID() {
    m_extenderPID.reset();
  }

  // use this ONLY to set the current position as zero
  public void zero() {
    m_motor.setSelectedSensorPosition(0.0);
  }

  // remove the soft limiter and have manual override
  public void removeLimiter() {
    m_manualEnabled = true;
  }

  // enable the soft limiter and go back to default controls
  public void enableLimiter() {
    m_manualEnabled = false;
  }

  public boolean isManual() {
    return m_manualEnabled;
  }

  // called in Robot by button binds
  public void setGoal(double distance) {
    m_setpoint = distance;
  }

  public double getGoal() {
    return m_setpoint;
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
    m_enabled = true;
  }
  
  // disables the extender
  public void disable() {
    m_enabled = false;
    m_motor.set(0);
  }

  // tells the motor object to drive at a speed that the PID sets the motorPower to be
  public void setOutput(double motorPower) {
    //if (motorPower > 0 || !limitSwitch.get()) {
      m_motor.set(MathUtil.clamp(motorPower, Constants.extender.kMotorClampDown, Constants.extender.kMotorClampUp));
    //}
  }

  // checks to see if the extender is fully compressed
  public boolean compressionLimitSwitch() {
    return m_limitSwitch.get();
  }

  public String getSide() {
    return m_side;
  }

  public void setSide(boolean isLeft) {
    if(isLeft) {
      m_side = "Left";
    } else {
      m_side = "Right";
    }
  }

  public boolean isEnabled(){
    return m_enabled;
  }

  public double getVelocity() {
    return m_motor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {

    if (m_manualEnabled) {
    
    if (m_side.equals("Left")) {
      if (Operator.operator.get(Axis.LEFT_Y) > 0.1) {
        setOutput(-0.2);
        m_enabled = false;
      } else if (Operator.operator.get(Axis.LEFT_Y) < -0.1) {
        setOutput(0.2);
        m_enabled = false;
      } else {
        setOutput(0);
      }
    }

    if (m_side.equals("Right")) {
      if (Operator.operator.get(Axis.RIGHT_Y) > 0.1) {
        setOutput(-0.2);
        m_enabled = false;
      } else if (Operator.operator.get(Axis.RIGHT_Y) < -0.1) {
        setOutput(0.2);
        m_enabled = false;
      } else {
        setOutput(0);
      }
    }

    }

    if(m_enabled) {
      // motor power is set to the extender pid's calculation
      setOutput(m_extenderPID.calculate(currentExtensionRaw(), m_setpoint));
    }
  }
} 