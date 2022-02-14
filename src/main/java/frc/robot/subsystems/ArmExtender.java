package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerFactory;
import frc.robot.robotConstants.extenderArm.TraversoExtenderArmConstants;


public class ArmExtender extends SubsystemBase {
  TraversoExtenderArmConstants constants = new TraversoExtenderArmConstants();
  private boolean enabled = true;
  private final WPI_TalonFX m_motor;
  private String smartDashText;

  // TODO: Change the PID of the extender!
  private PIDController extenderPID = new PIDController(0.0002, 0.0, 0.0);
  
  private double setpoint;

  // it was requested to use multiple objects for the extender because one might fail
  public ArmExtender(boolean left) {
    // if the arm is left, the tick value is inverted && objects are assigned correctly
    if (left) {
      m_motor = ControllerFactory.createTalonFX(constants.kLeftExtenderPort);
      smartDashText = "Current Extension (Left)";
    }
    else {
      // otherwise, just assign the motor object to the right
      m_motor = ControllerFactory.createTalonFX(constants.kRightExtenderPort);
      smartDashText = "Current Extension (Right)";

    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    // TODO: Update this max reverse limit!
    m_motor.configReverseSoftLimitThreshold(-5000, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    // TODO: Update this max forward limit!
    m_motor.configForwardSoftLimitThreshold(5000, 10);

    // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
    m_motor.setSelectedSensorPosition(0.0);

    // so that the limiters are enabled
    m_motor.configForwardSoftLimitEnable(true, 10);
    m_motor.configReverseSoftLimitEnable(true, 10);
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
    System.out.println();
    enabled = true;
  }
  
  // disables the extender
  public void disable() {
    enabled = false;
    m_motor.set(0);
  }

  // tells the motor object to drive at a speed that the PID sets the motorPower to be
  public void setOutput(double motorPower) {
    m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  @Override
  public void periodic() {
    // sets the PID to be the motorPower
    if(enabled) {
      setOutput(extenderPID.calculate(currentExtension(), setpoint));
    }
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber(smartDashText, currentExtension());
  }
  
} 
