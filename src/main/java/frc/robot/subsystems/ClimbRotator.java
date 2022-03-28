package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;
// import frc.robot.util.LimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbRotator extends SubsystemBase {
  private boolean enabled = true;
  private final DutyCycleEncoder encoder;
  private final WPI_TalonFX m_motor;
  public final String side;
  private boolean left;

  private double setpoint = Constants.rotator.kNinetyDeg;
  private double encoderOffset;

  public PIDController armPID = new PIDController(Constants.rotator.kP , Constants.rotator.kI , Constants.rotator.kD);
  // TODO: Check if we're using limit switches for the rotator and if not, remove these variables and the unneeded import <3
  // private LimitSwitch limitSwitchLower, limitSwitchUpper;

  public ClimbRotator(boolean isLeft) {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (isLeft) {
      encoder = new DutyCycleEncoder(Constants.rotator.kArmLeftEncoder); // initializes the through bore
      m_motor = ControllerFactory.createTalonFX(Constants.rotator.kArmLeftMotor , Constants.rotator.kSupplyCurrentLimit, Constants.rotator.kSupplyTriggerThreshold, Constants.rotator.kSupplyTriggerDuration, Constants.rotator.kNeutral); // initializes the motor
      side = "Left"; // the direction for shuffleboard's use
      m_motor.setInverted(true); // inverts the motor
      encoderOffset = Constants.rotator.kArmLeftEncoderOffset; // sets an offset for the encoder

     // limitSwitchLower = new LimitSwitch(Constants.rotator.kLeftLimitSwitchLower , Constants.rotator.kLimitSwitchDebouncer);
     // limitSwitchUpper = new LimitSwitch(Constants.rotator.kLeftLimitSwitchUpper , Constants.rotator.kLimitSwitchDebouncer);
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      encoder = new DutyCycleEncoder(Constants.rotator.kArmRightEncoder); // initializes the through bore
      m_motor = ControllerFactory.createTalonFX(Constants.rotator.kArmRightMotor , Constants.rotator.kSupplyCurrentLimit, Constants.rotator.kSupplyTriggerThreshold, Constants.rotator.kSupplyTriggerDuration, Constants.rotator.kNeutral); // initializes the motor
      side = "Right"; // the direction for shuffleboard's use
      encoderOffset = Constants.rotator.kArmRightEncoderOffset; // sets an offset for the encoder

     // limitSwitchLower = new LimitSwitch(Constants.rotator.kRightLimitSwitchLower , Constants.rotator.kLimitSwitchDebouncer);
     // limitSwitchUpper = new LimitSwitch(Constants.rotator.kRightLimitSwitchUpper , Constants.rotator.kLimitSwitchDebouncer);
    }
   
    left = isLeft;

    armPID.reset();

    // set the tolerance allowed for the PID
    armPID.setTolerance(Constants.rotator.kArmTolerance);

    //use this to calibrate rotators and then look in smartdashboard
    // setEncoder(123);
    // SmartDashboard.putNumber(side + " offset", encoderOffset);
  }

  @Override
  public void periodic() {
    
    // SmartDashboard.putNumber(direction + " rotator angle raw", currentAngleRaw());
    // SmartDashboard.putNumber(direction + " rotator angle", currentAngle());
    // SmartDashboard.putNumber(direction + " rotator offset", encoderOffset);
    if(enabled) {
      // set the arm power according to the PID
      setOutput(armPID.calculate(currentAngle(), setpoint));
    } else {
      m_motor.set(0);
    }
  }

  public double currentAngleRaw() {
    return encoder.get();
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    if(left) {
      return (-(currentAngleRaw() * Constants.rotator.kArmDegreeMultiple) + encoderOffset) % 360;
    } else {
      return (currentAngleRaw() * Constants.rotator.kArmDegreeMultiple + encoderOffset) % 360;
    }
  }

  // 80 is all the way forward and  125 is all the way back
  public void setEncoder(double angle) { 
    if (left) {
      encoderOffset = angle + encoder.get() * Constants.rotator.kArmDegreeMultiple;
    }
    else {
      encoderOffset = angle - encoder.get() * Constants.rotator.kArmDegreeMultiple;
    }
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
    return (currentAngle() < setpoint + Constants.rotator.kArmTolerance && currentAngle() > setpoint - Constants.rotator.kArmTolerance);
  }

  //enables PID
  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
    // if the subsystem is disabled, do not spin the motor
    m_motor.set(0);
  }

  public void setOutput(double motorPower){
    m_motor.set(MathUtil.clamp(motorPower, -Constants.rotator.kMotorClamp, Constants.rotator.kMotorClamp));
  }

  // sets PID Goal
  public void setGoal(double goal){
    armPID.reset();
    setpoint = goal;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public String getSide() {
      return side;
  }

  public double getSetPoint() {
      return setpoint;
  }
}