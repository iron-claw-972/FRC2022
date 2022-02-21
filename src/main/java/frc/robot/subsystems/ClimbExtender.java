package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;


public class ClimbExtender extends SubsystemBase {
  TraversoClimbExtenderConstants constants = new TraversoClimbExtenderConstants();
  private boolean enabled = false;
  private final WPI_TalonFX m_motor;
  private String direction;
  private double motorClamp = constants.kMotorClampOffLoad;
  private boolean left;

  private PIDController extenderPID = new PIDController(constants.kP, constants.kI, constants.kD);
  
  private double setpoint;

  // it was requested to use multiple objects for the extender because one might fail
  public ClimbExtender(boolean isLeft) {
    // if the arm is left, the tick value is inverted && objects are assigned correctly
    if (isLeft) {
      m_motor = ControllerFactory.createTalonFX(constants.kLeftExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast); // initializes the motor
      direction = "Left"; // the direction for shuffleboard's use
      m_motor.setInverted(true);
    }
    else {
      // otherwise, just assign the motor object to the right
      m_motor = ControllerFactory.createTalonFX(constants.kRightExtenderPort, constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast); // initializes the motor
      direction = "Right"; // the direction for shuffleboard's use
    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    m_motor.configReverseSoftLimitThreshold(0, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    // TODO: Update this max forward limit!
    m_motor.configForwardSoftLimitThreshold(5000, 10);

    // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
    m_motor.setSelectedSensorPosition(0.0);

    // so that the limiters are enabled
    // TODO: If the motors don't move, CHECK TO SEE IF THE LIMITER IS TOO LOW!
    m_motor.configForwardSoftLimitEnable(false, 10);
    m_motor.configReverseSoftLimitEnable(false, 10);

    // set the PID's tolerance
    extenderPID.setTolerance(constants.kExtenderTolerance);

    left = isLeft;
  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- 10), return true, otherwise return false
    return extenderPID.atSetpoint();
  }

  // called in RobotContainer by button binds
  public void set(double distance) {
    setpoint = distance;
  }

  // returns the current extension in inches
  public double currentExtension() {
    if(left) {
      return m_motor.getSelectedSensorPosition() * constants.kExtenderTickMultiple;
    }
    else {
      return m_motor.getSelectedSensorPosition() * constants.kExtenderTickMultiple;
    }
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
    m_motor.set(MathUtil.clamp(motorPower, -motorClamp, motorClamp));
  }

  @Override
  public void periodic() {
    if(enabled) {
      // motor power is set to the extenderpid's calculation
      setOutput(extenderPID.calculate(currentExtension(), setpoint));
    }
  }

  public void loadExtenderShuffleboard() {
    SmartDashboard.putData("Climb Extender PID", extenderPID);
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber(direction + " Extension", currentExtension());
    // a pop-up in shuffleboard that states if the extender is on/off
    SmartDashboard.putBoolean(direction + " Extender", enabled);
  }
} 
