package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.climbRotator.MarinusClimbRotatorConstants;
import frc.robot.util.LimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbRotator extends SubsystemBase {
  MarinusClimbRotatorConstants constants = new MarinusClimbRotatorConstants();

  private boolean enabled = false;
  private final DutyCycleEncoder encoder;
  // private final WPI_TalonFX m_motor;
  public final String side;
  private boolean left;

  private double setPoint = 90;
  private double encoderOffset;

  public PIDController armPID = new PIDController(constants.kP , constants.kI , constants.kD);
  private LimitSwitch limitSwitchLower, limitSwitchUpper;

  public ClimbRotator(boolean isLeft) {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (isLeft) {
      encoder = new DutyCycleEncoder(constants.kArmLeftEncoder); // initializes the through bore
      // m_motor = ControllerFactory.createTalonFX(constants.kArmLeftMotor , constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kNeutral); // initializes the motor
      side = "Left"; // the direction for shuffleboard's use
      // m_motor.setInverted(true); // inverts the motor
      encoderOffset = constants.kArmLeftEncoderOffset; // sets an offset for the encoder

      limitSwitchLower = new LimitSwitch(constants.kLeftLimitSwitchLower , constants.kLimitSwitchDebouncer);
      limitSwitchUpper = new LimitSwitch(constants.kLeftLimitSwitchUpper , constants.kLimitSwitchDebouncer);
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      encoder = new DutyCycleEncoder(constants.kArmRightEncoder); // initializes the through bore
      // m_motor = ControllerFactory.createTalonFX(constants.kArmRightMotor , constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kNeutral); // initializes the motor
      side = "Right"; // the direction for shuffleboard's use
      encoderOffset = constants.kArmRightEncoderOffset; // sets an offset for the encoder

      limitSwitchLower = new LimitSwitch(constants.kRightLimitSwitchLower , constants.kLimitSwitchDebouncer);
      limitSwitchUpper = new LimitSwitch(constants.kRightLimitSwitchUpper , constants.kLimitSwitchDebouncer);
    }
   
    left = isLeft;

    armPID.reset();

    // set the tolerance allowed for the PID
    armPID.setTolerance(constants.kArmTolerance);
    // SmartDashboard.putData(direction + " rot", armPID);

    //setEncoder(120);
   // SmartDashboard.putNumber(side + " offset", encoderOffset);
  }

  @Override
  public void periodic() {
    
    // SmartDashboard.putNumber(direction + " rotator angle raw", currentAngleRaw());
    // SmartDashboard.putNumber(direction + " rotator angle", currentAngle());
    // SmartDashboard.putNumber(direction + " rotator offset", encoderOffset);
    if(enabled) {
      // set the arm power according to the PID
      // setOutput(armPID.calculate(currentAngle(), setPoint));
    }
  }

  public double currentAngleRaw() {
    return encoder.get();
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    if(left) {
      return (-(currentAngleRaw() * constants.kArmDegreeMultiple) + encoderOffset) % 360;
    } else {
      return (currentAngleRaw() * constants.kArmDegreeMultiple + encoderOffset) % 360;
    }
  }

  // 80 is all the way forward and  125 is all the way back
  public void setEncoder(double angle) { 
    if (left) {
      encoderOffset = angle + encoder.get() * constants.kArmDegreeMultiple;
    }
    else {
      encoderOffset = angle - encoder.get() * constants.kArmDegreeMultiple;
    }
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
    return armPID.atSetpoint();// || limitSwitchLower.risingEdge() || limitSwitchUpper.risingEdge();
  }

  //enables PID
  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
    // if the subsystem is disabled, do not spin the motor
    // m_motor.set(0);
  }

  public void setOutput(double motorPower){
    // m_motor.set(MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  // sets PID Goal
  public void setGoal(double goal){
    setPoint = goal;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public String getSide() {
      return side;
  }

  public double getSetPoint() {
      return setPoint;
  }
}