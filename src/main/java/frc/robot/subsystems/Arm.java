package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private boolean enabled = true;
  private final DutyCycleEncoder dceEncoder;
  private final WPI_TalonFX m_motor;
  private boolean isLeft = false;

  private double setpoint = 0;

  private PIDController armPID = new PIDController(0.02, 0, 0);

  public Arm(boolean left) {

    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (left) {
      dceEncoder = new DutyCycleEncoder(ArmConstants.kArmLeftEncoder);
      m_motor = ControllerFactory.createTalonFX(ArmConstants.kArmLeftMotor);
    }
    // otherwise, use the normal encoder value and set the motor ports to the right
    else {
      dceEncoder = new DutyCycleEncoder(ArmConstants.kArmRightEncoder);
      m_motor = ControllerFactory.createTalonFX(ArmConstants.kArmRightMotor);
    }
    // store the left boolean in isLeft
    isLeft = left;
  }

  public double currentTickVal() {
    if(isLeft) {
      return -dceEncoder.get();
    }
    return dceEncoder.get();
  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- tolerance), return true, otherwise return false
    return currentTickVal() >= (setpoint / ArmConstants.kArmDegreeMultiple) - ArmConstants.kArmTolerance
    && currentTickVal() <= (setpoint / ArmConstants.kArmDegreeMultiple) + ArmConstants.kArmTolerance;
  }

  // called in RobotContainer by button binds
  public void set(double distance){
    setpoint = distance;
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
  }

  @Override
  public void periodic() {
    if(enabled) {
      // if the current tick value falls within the setpoint by 10 ticks
      if (reachedSetpoint()) {
        m_motor.set(0);
      }
      else {
        // set the arm power according to a PID
        m_motor.set(armPID.calculate(currentTickVal(), setpoint));
      }
    }
    else {
      // if the subsystem is disabled, do not spin the motor
      m_motor.set(0);
    }
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Angle (Degrees)", dceEncoder.get() * ArmConstants.kArmDegreeMultiple);
  }
}