package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.climbArm.TraversoClimbArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbArm extends SubsystemBase {
  TraversoClimbArmConstants constants = new TraversoClimbArmConstants();

  private boolean enabled = false;
  private final DutyCycleEncoder dce;
  private final WPI_TalonFX m_motor;
  boolean storedLeft;

  private double setpoint = 0;

  private PIDController armPID = new PIDController(0.007, 0.0008, 0.0005);

  public ClimbArm(boolean left) {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (left) {
      dce = new DutyCycleEncoder(constants.kArmLeftEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmLeftMotor);
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      dce = new DutyCycleEncoder(constants.kArmRightEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmRightMotor);
    }
    // store the left boolean in storedLeft
    storedLeft = left;
    armPID.setTolerance(constants.kArmTolerance);
  }

  // returns the current angle of the duty cycle encoder
  public double currentAngle() {
    if(storedLeft) {
      return -(dce.get()*constants.kArmDegreeMultiple-constants.kArmZeroEncoderDegrees);
    }
    return dce.get()*constants.kArmDegreeMultiple-constants.kArmZeroEncoderDegrees;
  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- tolerance), return true, otherwise return false
    return armPID.atSetpoint();
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
    m_motor.set(0);
    // if the subsystem is disabled, do not spin the motor
  }

  public void setOutput(double motorPower){
    m_motor.set(ControlMode.PercentOutput, constants.kFlipped * MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  @Override
  public void periodic(){
    if(enabled) {
      // set the arm power according to a PID
      setOutput(armPID.calculate(currentAngle(), setpoint));
    }


    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Angle (Degrees)", dce.get() * constants.kArmDegreeMultiple);
    System.out.println(currentAngle());
  }
}