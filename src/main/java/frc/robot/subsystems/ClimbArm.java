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

  private double setPoint = 90;
  private double encoderOffset;

  private PIDController armPID = new PIDController(constants.kOffLoadP , constants.kOffLoadI , constants.kOffLoadD);
  
  public ClimbArm(boolean left) {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (left) {
      dce = new DutyCycleEncoder(constants.kArmLeftEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmLeftMotor);
      m_motor.setInverted(true);
      encoderOffset = constants.kArmLeftEncoderOffset;
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      dce = new DutyCycleEncoder(constants.kArmRightEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmRightMotor);
      encoderOffset = constants.kArmRightEncoderOffset;
    }
    // store the left boolean in storedLeft
    storedLeft = left;
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

      // set the arm power according to a PID
      setOutput(armPID.calculate(currentAngle(), setPoint));
    }

    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Angle (Degrees)", currentAngle());
    // System.out.println(currentAngle());

  }

  public double currentAngleRaw() {
    return dce.get();
  }

  // returns the current angle of the duty cycle encoder with offset accounted for
  public double currentAngle() {
    if(storedLeft) {
      return -(dce.get() * constants.kArmDegreeMultiple + encoderOffset);
    } else {
      return dce.get() * constants.kArmDegreeMultiple + encoderOffset;
    }
  }

  // 80 is all the way forward and  125 is all the way back
  public void setEncoder(double angle) { 
    encoderOffset = angle // constants.kArmDegreeMultiple
          - dce.get() * constants.kArmDegreeMultiple;
    System.out.println("set encoder");

  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- tolerance), return true, otherwise return false
    return armPID.atSetpoint();
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
    m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
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