package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.climbRotator.TraversoClimbRotatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbRotator extends SubsystemBase {
  TraversoClimbRotatorConstants constants = new TraversoClimbRotatorConstants();

  private boolean enabled = false;
  private final DutyCycleEncoder encoder;
  private final WPI_TalonFX m_motor;
  private String smartDashText;
  boolean storedLeft;

  // this is set by the method "set" called later on
  private double setpoint = 0;

  private PIDController armPID = new PIDController(0.007, 0.0008, 0.0005);

  public ClimbRotator(boolean left) {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (left) {
      encoder = new DutyCycleEncoder(constants.kArmLeftEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmLeftMotor);
      smartDashText = "Current Angle (Left)";
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      encoder = new DutyCycleEncoder(constants.kArmRightEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmRightMotor);
      smartDashText = "Current Angle (Right)";
    }
    // store the left boolean in storedLeft
    storedLeft = left;

    // set the tolerance allowed for the PID
    armPID.setTolerance(constants.kArmTolerance);
  }

  // returns the current angle of the duty cycle encoder
  public double currentAngle() {
    if(storedLeft) {
      // if it's the left motor, give the opposite value
      return -(encoder.get()*constants.kArmDegreeMultiple-constants.kArmZeroEncoderDegrees);
    }
    // if it's the right motor, give the normal value
    return encoder.get()*constants.kArmDegreeMultiple-constants.kArmZeroEncoderDegrees;
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
    return armPID.atSetpoint();
  }

  // called in RobotContainer by button binds
  public void set(double distance) {
    setpoint = distance;
  }

  // allows the motor to be spun
  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
    // if the subsystem is disabled, do not spin the motor
    m_motor.set(0);
  }

  public void setOutput(double motorPower) {
    m_motor.set(ControlMode.PercentOutput, constants.kFlipped * MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  @Override
  public void periodic() {
    if(enabled) {
      // set the arm power according to a PID
      setOutput(armPID.calculate(currentAngle(), setpoint));
    }
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber(smartDashText, currentAngle());
  }
  
}