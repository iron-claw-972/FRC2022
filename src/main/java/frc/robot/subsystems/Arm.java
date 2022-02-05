package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ArmConstants;

public class Arm {
  //set to defult radian angle probly should set to
  double armGoal = 0;
  
  private final WPI_TalonFX m_leftMotor  = ControllerFactory.createTalonFX(ArmConstants.kLeftMotorPort);
  private final WPI_TalonFX m_rightMotor = ControllerFactory.createTalonFX(ArmConstants.kRightMotorPort);
  

  private final PhoenixMotorControllerGroup m_motors = new PhoenixMotorControllerGroup(m_leftMotor, m_rightMotor);
  
  // arm encoder
  private final TalonEncoder m_leftEncoder = new TalonEncoder(m_leftMotor);
  private final TalonEncoder m_rightEncoder = new TalonEncoder(m_rightMotor);
  

  public Arm(){
    m_rightMotor.setInverted(true);
  }
  
  double maxOutput = 0.1;
  double maxError = 2;
  double error;
  double derivative;
  double errorPrior;
  double pGain = 0.1;
  double iGain = 0.1;
  double dGain = 0.1;

  double outPutVal;
  double integralPrior;
  double integral =0;
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

    m_motors.set(outPutVal);
  }
  
  public double getPosition(){
    // System.out.println(m_leftEncoder.getDistance();
    return m_leftEncoder.getDistance() * ArmConstants.kEncoderRadiansPerPulse;
    // might need more operations in order to get the angle ready for PID
  }
  
  public double getVelocity(){
      // System.out.println(m_leftEncoder.getRate());
      return m_leftEncoder.getRate();
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
  
    
}
