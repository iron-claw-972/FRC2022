package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ctre_shims.TalonEncoderSim;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
  //set to default radian angle probably should set to pi/2
  double armGoal = 0;
  
  // private final WPI_TalonFX m_leftMotor  = ControllerFactory.createTalonFX(ArmConstants.kLeftMotorPort);
  // private final WPI_TalonFX m_rightMotor = ControllerFactory.createTalonFX(ArmConstants.kRightMotorPort);
  private final WPI_TalonSRX motor  = ControllerFactory.createTalonSRX(3);
  

  // private final PhoenixMotorControllerGroup m_motors = new PhoenixMotorControllerGroup(m_leftMotor, m_rightMotor);
  
  // arm encoder
  // private final TalonEncoder m_leftEncoder = new TalonEncoder(m_leftMotor);
  // private final TalonEncoder m_rightEncoder = new TalonEncoder(m_rightMotor);
  
  // private final WPI_TalonFX m_motor;
  // private final TalonEncoder m_encoder;
  // private double setpoint = ArmConstants.kRotatorSetpoint;

  /*
  public Arm(int port, boolean left) {
    m_motor = ControllerFactory.createTalonFX(port);
    m_encoder= new TalonEncoder(m_motor);

    if (left) {
      // so that encoder values aren't negative
      m_motor.setSensorPhase(true);
      // so that the arm doesn't spin in an opposing direction
      m_motor.setInverted(true);
    }

    // the lowest tick limit is 0, and must be checked every 10 milliseconds
    m_motor.configReverseSoftLimitThreshold(0, 10);

    // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
    m_motor.configForwardSoftLimitThreshold(ArmConstants.kRotatorMaxArmTicks, 10);

    // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
    // TODO: Make this better.
    m_encoder.reset();

    // so that the limiters are enabled
    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
  }
  
  
  
  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- 10), return true, otherwise return false
    return(m_encoder.getDistance() >= (setpoint / ArmConstants.kRotatorTickMultiple) - ArmConstants.kRotatorTolerance
    && m_encoder.getDistance() <= (setpoint / ArmConstants.kRotatorTickMultiple) + ArmConstants.kRotatorTolerance);
  }
  
  
  // called in RobotContainer by button binds
  public void set(double distance){
    setpoint = distance;
  }


  @Override
  public void periodic(){
    if (reachedSetpoint() == false) {
      // sets the motor to go to a setpoint
      // the setpoint is tick value
      m_motor.set(ArmConstants.rotatorPID.calculate(m_encoder.getDistance(), setpoint));

      // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
      SmartDashboard.putNumber("Current Extension (Inches)", m_encoder.getDistance() * ArmConstants.kRotatorTickMultiple);

      // so we know the value
      // System.out.println(setpoint);
      System.out.println(m_encoder.getDistance());
    }
  }
  */
  
  //Michael's old code that may or may not need to be reused
  public Arm(){
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    // m_rightMotor.setInverted(true);
  }
  
  double maxOutput = 0.1;
  double maxError = Math.PI / 90.0;
  double error;
  double derivative;
  double errorPrior;
  double pGain = SmartDashboard.getNumber("P", 0.1);
  double iGain = SmartDashboard.getNumber("I", 0);
  double dGain = SmartDashboard.getNumber("D", 0);

  //private PIDController pid = new PIDController(pGain, iGain, dGain);

  double outPutVal;
  double integralPrior;
  double integral = 0;
  double iterationTime = 0.001;

  public void posPID() {

    error = armGoal - getPosition();
    integral = integralPrior + error * iterationTime;
    derivative = (error - errorPrior) / iterationTime;

    outPutVal = pGain *error + iGain *integral + dGain *derivative;
    outPutVal = outPutVal * Math.cos(getPosition());
    
    errorPrior = error;
    integralPrior = integral;

    /*
    if (outPutVal > maxOutput){
      outPutVal = maxOutput;
    }if (outPutVal < -maxOutput) {
      outPutVal = -maxOutput;
    }if (maxError > error && error > maxError) {
      outPutVal = 0;
    }
    */

    System.out.println("power: " + outPutVal);
    motor.set(ControlMode.PercentOutput, outPutVal);
  }
  
  public double getPosition() {
    // System.out.println(m_leftEncoder.getDistance();
    // return m_leftEncoder.getDistance() * ArmConstants.kEncoderRadiansPerPulse;
    return motor.getSelectedSensorPosition() * ArmConstants.kEncoderRadiansPerPulse;
    // 2033 == pi
    // might need more operations in order to get the angle ready for PID
  }
  
  public double getVelocity(){
      // System.out.println(m_leftEncoder.getRate());
      // return m_leftEncoder.getRate();
      return motor.getSelectedSensorVelocity();
  }

  public void moveGoal(double target){
    armGoal = armGoal + target;
  }
  
  public void setGoal(double target){
    armGoal = target;
  }

  public double getGoal(){
      return armGoal;
  }

  public boolean atGoal(){
    return (error < errorPrior && errorPrior < maxError);
  }
  
  public void setEncoder(double radians){
    motor.setSelectedSensorPosition(radians/(Math.PI*2)*ArmConstants.kEncoderResolution);
  }
    
}
