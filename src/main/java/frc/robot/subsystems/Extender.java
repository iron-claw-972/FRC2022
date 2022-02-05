package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ExtenderConstants;


public class Extender extends SubsystemBase{
  private final WPI_TalonFX m_motor;
  private double setpoint = ExtenderConstants.kExtenderSetpoint;

  public Extender(int port, boolean left) {
    m_motor = ControllerFactory.createTalonFX(port);

    if (left) {
      // so that encoder values aren't negative
      m_motor.setSensorPhase(true);
      // so that the arm doesn't spin in an opposing direction
      m_motor.setInverted(true);
    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    m_motor.configReverseSoftLimitThreshold(0, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    m_motor.configForwardSoftLimitThreshold(ExtenderConstants.kExtenderMaxArmTicks, 10);

    // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
    // TODO: Make this better.
    m_motor.setSelectedSensorPosition(0.0);

    // so that the limiters are enabled
    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- 10), return true, otherwise return false
    return(m_motor.getSelectedSensorPosition() >= (setpoint / ExtenderConstants.kExtenderTickMultiple) - ExtenderConstants.kExtenderTolerance
    && m_motor.getSelectedSensorPosition() <= (setpoint / ExtenderConstants.kExtenderTickMultiple) + 10);
  }

  // called in RobotContainer by button binds
  public void set(double distance){
    setpoint = distance;
  }


  @Override
  public void periodic(){
    if (reachedSetpoint()) {
      // sets the motor to go to a setpoint
      // the setpoint is tick value
      m_motor.set(ExtenderConstants.extenderPID.calculate(m_motor.getSelectedSensorPosition(), setpoint));

      // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
      SmartDashboard.putNumber("Current Extension (Inches)", m_motor.getSelectedSensorPosition() * ExtenderConstants.kExtenderTickMultiple);

      // so we know the value
      System.out.println(setpoint);
    }
  }
} 
