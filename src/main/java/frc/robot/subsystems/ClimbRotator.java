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

  private double setPoint = 90;
  private double encoderOffset;

  private PIDController armPID = new PIDController(constants.kOffLoadP , constants.kOffLoadI , constants.kOffLoadD);
  
  public ClimbRotator(boolean left) {
    // if the arm is left, the encoder value is inverted && the objects are assigned correctly
    if (left) {
      encoder = new DutyCycleEncoder(constants.kArmLeftEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmLeftMotor);
      smartDashText = "Current Angle (Left)";
      m_motor.setInverted(true);
      encoderOffset = constants.kArmLeftEncoderOffset;
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      encoder = new DutyCycleEncoder(constants.kArmRightEncoder);
      m_motor = ControllerFactory.createTalonFX(constants.kArmRightMotor);
      smartDashText = "Current Angle (Right)";
      encoderOffset = constants.kArmRightEncoderOffset;
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

      // set the arm power according to a PID
      setOutput(armPID.calculate(currentAngle(), setPoint));
      SmartDashboard.putNumber(smartDashText, currentAngle());
    }

    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber("Current Angle (Degrees)", currentAngle());
    // System.out.println(currentAngle());

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
    return armPID.atSetpoint();
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