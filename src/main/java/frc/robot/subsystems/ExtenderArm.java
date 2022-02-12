package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerFactory;
import frc.robot.robotConstants.extenderArm.TraversoExtenderArmConstants;


public class ExtenderArm extends SubsystemBase{
  TraversoExtenderArmConstants constants = new TraversoExtenderArmConstants();
  private boolean enabled = true;
  private final WPI_TalonFX m_motor;

  private PIDController extenderPID = new PIDController(0.0, 0.0, 0.01);
  
  private double setpoint = 0;

  // this is initialized in RobotContainer where it uses the respective constants
  // it was requested to use multiple objects for the extender because one might fail
  public ExtenderArm(boolean left) {
    // if the arm is left, the tick value is inverted && objects are assigned correctly
    if (left) {
      m_motor = ControllerFactory.createTalonFX(constants.kLeftExtenderPort);
      // so that encoder values aren't negative
      m_motor.setSensorPhase(true);
      // so that the extender doesn't extend in an opposing direction
      m_motor.setInverted(true);
    }
    else {
      // otherwise, just assign the motor object to the right
      m_motor = ControllerFactory.createTalonFX(constants.kRightExtenderPort);
    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    m_motor.configReverseSoftLimitThreshold(0, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    m_motor.configForwardSoftLimitThreshold((int)constants.kExtenderMaxArmTicks, 10);

    // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
    // TODO: Make this better.
    m_motor.setSelectedSensorPosition(0.0);

    // so that the limiters are enabled
    m_motor.configForwardSoftLimitEnable(true, 10);
    m_motor.configReverseSoftLimitEnable(true, 10);
  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- 10), return true, otherwise return false
    return(m_motor.getSelectedSensorPosition() >= (setpoint / constants.kExtenderTickMultiple) - constants.kExtenderTolerance
    && m_motor.getSelectedSensorPosition() <= (setpoint / constants.kExtenderTickMultiple) + constants.kExtenderTolerance);
  }

  // called in RobotContainer by button binds
  public void set(double distance){
    setpoint = distance;
  }

  public double currentExtension() {
    return m_motor.getSelectedSensorPosition() * constants.kExtenderTickMultiple;
  }

  public void enable() {
    enabled = true;
  }
  
  public void disable() {
    enabled = false;
    m_motor.set(0);
  }

  @Override
  public void periodic(){
    if(enabled) {
      set(extenderPID.calculate(currentExtension(), setpoint));
    }
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Extension (Inches)", m_motor.getSelectedSensorPosition() * constants.kExtenderTickMultiple);
  }
} 