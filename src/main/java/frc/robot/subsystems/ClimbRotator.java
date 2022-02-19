package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.climbRotator.TraversoClimbRotatorConstants;
import frc.robot.util.LimitSwitch;
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
  private String direction;
  boolean storedLeft;

  private double setPoint = 90;
  private double encoderOffset;

  private PIDController armPID = new PIDController(constants.kOffLoadP , constants.kOffLoadI , constants.kOffLoadD);
  private LimitSwitch limitSwitchLower;
  private LimitSwitch limitSwitchUpper;

  public ClimbRotator(boolean left) {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (left) {
      encoder = new DutyCycleEncoder(constants.kArmLeftEncoder); // initializes the through bore
      m_motor = ControllerFactory.createTalonFX(constants.kArmLeftMotor); // initializes the motor
      direction = "(Left)"; // the direction for shuffleboard's use
      m_motor.setInverted(true); // inverts the motor
      encoderOffset = constants.kArmLeftEncoderOffset; // sets an offset for the encoder

      limitSwitchLower = new LimitSwitch(constants.kLeftLimitSwitchLower , constants.kLimitSwitchDebouncer);
      limitSwitchUpper = new LimitSwitch(constants.kLeftLimitSwitchUpper , constants.kLimitSwitchDebouncer);
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      encoder = new DutyCycleEncoder(constants.kArmRightEncoder); // initializes the through bore
      m_motor = ControllerFactory.createTalonFX(constants.kArmRightMotor); // initializes the motor
      direction = "(Right)"; // the direction for shuffleboard's use
      encoderOffset = constants.kArmRightEncoderOffset; // sets an offset for the encoder

      limitSwitchLower = new LimitSwitch(constants.kRightLimitSwitchLower , constants.kLimitSwitchDebouncer);
      limitSwitchUpper = new LimitSwitch(constants.kRightLimitSwitchUpper , constants.kLimitSwitchDebouncer);
    }
    // store the left boolean in storedLeft
    storedLeft = left;

    // set the tolerance allowed for the PID
    armPID.setTolerance(constants.kArmTolerance);
    this.offLoad();
    //Puts PID values on shuffle board for tuning the PID (to be commented out later)
    SmartDashboard.putNumber("P", constants.kOffLoadP);
    SmartDashboard.putNumber("I", constants.kOffLoadI);
    SmartDashboard.putNumber("D", constants.kOffLoadD);
    SmartDashboard.putNumber("set encoder", 80);
    SmartDashboard.putNumber("goal", 90);
    setEncoder(80);
  }

  @Override
  public void periodic() {

    if(enabled) {

      // gets PID values from shuffle board for tuning the PID (to be commented out later)
      armPID.setP(SmartDashboard.getNumber("P", constants.kOffLoadP));
      armPID.setI(SmartDashboard.getNumber("I", constants.kOffLoadI));
      armPID.setD(SmartDashboard.getNumber("D", constants.kOffLoadD));
      // setpoint = SmartDashboard.getNumber("goal", 0);

      // set the arm power according to the PID
      setOutput(armPID.calculate(currentAngle(), setPoint));

      
    }
    // SmartDashboard.putBoolean("limit switch", limitSwitch.get());
    // System.out.println(limitSwitch.get());
    

    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Angle " + direction, currentAngle());
    // System.out.println(currentAngle());
    // a pop-up in shuffleboard that states if the rotator is on/off
    SmartDashboard.putBoolean("Rotator On/Off" + direction, enabled);
  }

  public double currentAngleRaw() {
    return encoder.get();
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    if(storedLeft) {
      return -(encoder.get() * constants.kArmDegreeMultiple + encoderOffset);
    } else {
      return encoder.get() * constants.kArmDegreeMultiple + encoderOffset;
    }
  }

  // 80 is all the way forward and  125 is all the way back
  public void setEncoder(double angle) { 
    encoderOffset = angle /*constants.kArmDegreeMultiple*/ - encoder.get() * constants.kArmDegreeMultiple;
  }

  public boolean reachedSetpoint() {
    // checks if the arm is at its setpoint
  
    // PLEASE NOTE:
    // LimitSwitch.java returns .get() as true WHEN THE ARM ISN'T BEYOND THE SETPOINT
    // it returns as FALSE when the limit has been exceeded

    // if the PID isn't at the setpoint, check if the upper limit switch
    if(armPID.atSetpoint() == false) {
      // if the upper limit switch returns as true (NOT AT SETPOINT), check the lower limit switch
      if(limitSwitchUpper.get()) {
        // if the lower limit switch returns as true (NOT AT SETPOINT), return as false (did not reach setpoint)
        if(limitSwitchLower.get()) {
          return false;
        }
        // if the upper limit switch hasn't reached the setpoint but the lower did, return true
        return true;
      }
      // if the PID didn't reach the setpoint but the upper limit switch did, return true
      return true;
    }
    // if the PID reached its setpoint, return true
    return true;
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
    m_motor.set(MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  public void offLoad(){
    armPID.setP(constants.kOffLoadP);
    armPID.setI(constants.kOffLoadI);
    armPID.setD(constants.kOffLoadD);
  }

  public void onLoad(){
    armPID.setP(constants.kOnLoadP);
    armPID.setI(constants.kOnLoadI);
    armPID.setD(constants.kOnLoadD);
  }

  // sets PID Goal
  public void setGoal(double goal){
    setPoint = goal;
  }

}