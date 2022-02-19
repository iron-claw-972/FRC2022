package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerFactory;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;


public class ClimbExtender extends SubsystemBase {
  TraversoClimbExtenderConstants constants = new TraversoClimbExtenderConstants();
  private boolean enabled = true;
  private final WPI_TalonFX m_motor;
  private String direction;

  // TODO: Change the PID of the extender!
  private PIDController extenderPID = new PIDController(constants.kOffLoadP, constants.kOffLoadI, constants.kOffLoadD);
  
  private double setpoint;

  // it was requested to use multiple objects for the extender because one might fail
  public ClimbExtender(boolean left) {
    // if the arm is left, the tick value is inverted && objects are assigned correctly
    if (left) {
      m_motor = ControllerFactory.createTalonFX(constants.kLeftExtenderPort); // initializes the motor
      direction = "(Left)"; // the direction for shuffleboard's use
      m_motor.setInverted(true);
    }
    else {
      // otherwise, just assign the motor object to the right
      m_motor = ControllerFactory.createTalonFX(constants.kRightExtenderPort); // initializes the motor
      direction = "(Right)"; // the direction for shuffleboard's use
    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    // TODO: Update this max reverse limit!
    m_motor.configReverseSoftLimitThreshold(0, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    // TODO: Update this max forward limit!
    m_motor.configForwardSoftLimitThreshold(5000, 10);

    // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
    m_motor.setSelectedSensorPosition(0.0);

    // so that the limiters are enabled
    m_motor.configForwardSoftLimitEnable(true, 10);
    m_motor.configReverseSoftLimitEnable(true, 10);

    // set the PID's tolerance
    extenderPID.setTolerance(constants.kExtenderTolerance);

    SmartDashboard.putNumber("P(e)", constants.kOffLoadP);
    SmartDashboard.putNumber("I(e)", constants.kOffLoadI);
    SmartDashboard.putNumber("D(e)", constants.kOffLoadD);
    SmartDashboard.putNumber("Goal(e)", 0);
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
    return m_motor.getSelectedSensorPosition() * constants.kExtenderTickMultiple;
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
    m_motor.set(MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  @Override
  public void periodic() {
    if(enabled) {
      extenderPID.setP(SmartDashboard.getNumber("P(e)", constants.kOnLoadP));
      extenderPID.setI(SmartDashboard.getNumber("I(e)", constants.kOnLoadI));
      extenderPID.setD(SmartDashboard.getNumber("D(e)", constants.kOnLoadD));

      setpoint = SmartDashboard.getNumber("Goal(e)", 0);

      loadCheck();
      
      // set the extender power according to the PID
      setOutput(extenderPID.calculate(currentExtension(), setpoint));
    }

    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("(e)Current Extension " + direction, currentExtension());
    // a pop-up in shuffleboard that states if the extender is on/off
    SmartDashboard.putBoolean("(e)On/Off " + direction, enabled);
    // a pop-up in shuffleboard that shows the tick value of the motor
    SmartDashboard.putNumber("(e)Current Raw Extension " + direction, currentExtensionRaw());
  }

  public void offLoad(){
    extenderPID.setP(constants.kOffLoadP);
    extenderPID.setI(constants.kOffLoadI);
    extenderPID.setD(constants.kOffLoadD);
  }

  public void onLoad(){
    extenderPID.setP(constants.kOnLoadP);
    extenderPID.setI(constants.kOnLoadI);
    extenderPID.setD(constants.kOnLoadD);
  }

  public void loadCheck() {
    if(setpoint < currentExtension()) {
      onLoad();
    }
    else {
      offLoad();
    }
  }
} 
