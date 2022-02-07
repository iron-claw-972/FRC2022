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
import frc.robot.ControllerFactory;
import frc.robot.Constants.ArmConstants;

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
  

  public Arm(){
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    // m_rightMotor.setInverted(true);
  }
  
  double maxOutput = 0.1;
  double maxError = Math.PI / 90.0;
  double error;
  double derivative;
  double errorPrior;
  double pGain = 0.5;
  double iGain = 0;
  double dGain = 0;

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
