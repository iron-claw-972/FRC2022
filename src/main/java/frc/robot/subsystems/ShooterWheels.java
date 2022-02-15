package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TraversoShooterWheelConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterWheels extends SubsystemBase {

  private final CANSparkMax m_wheelsMotor = new CANSparkMax(8, MotorType.kBrushless);

  private final CANSparkMax m_wheelsMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final PIDController m_wheelsPID = new PIDController(ShooterConstants.kBottomMotorP,
      ShooterConstants.kBottomMotorI, ShooterConstants.kBottomMotorD);
  private final SimpleMotorFeedforward ShooterWheelFeedForward = new SimpleMotorFeedforward(
      TraversoShooterWheelConstants.kS, TraversoShooterWheelConstants.kV,
      TraversoShooterWheelConstants.kA);

  private boolean PIDenabled = true;
  public static double motorSpeed = 0.0;

  public ShooterWheels() {
    m_wheelsPID.reset();
    m_wheelsPID.setSetpoint(motorSpeed);
    m_wheelsMotor2.follow(m_wheelsMotor);
  }

  @Override
  public void periodic() {
    if (PIDenabled) {
      updatePID();
    }
  }

  double speed = 0;
  double pidPow = 0;
  double pow = 0;

  public void updatePID() {
    double s = getEncoderVelocity();
    if (s != speed) {
      System.out.println("Speed: " + s);
      speed = s;
    }
    double p = m_wheelsPID.calculate(getEncoderVelocity(), motorSpeed);
    if (pidPow != p) {
      System.out.println("PID Pow: " + p);
      pidPow = p;
    }
    double k = ((12.0 / 5676.0) * motorSpeed + pow);
    if (k != pow) {
      System.out.println("Power: " + k);
      pow = k;
    }

    //m_wheelsMotor.set((12.0 / 5676.0) * motorSpeed + pow);

  }

  public void setSpeed(double speed) {
    PIDenabled = true;
    motorSpeed = speed;
  }

  public void setIntakeSpeed() {
    setSpeed(ShooterConstants.kShooterWheelsIntakeSpeed);
  }

  public void setOutakeSpeed() {
    setSpeed(ShooterConstants.kShooterWheelsFrontOutakeSpeed);
  }

  public void stop() {
    setSpeed(0);
    m_wheelsMotor.set(0);
    PIDenabled = false;
  }

  public boolean reachedSetpoint(double targetSpeed) {
    return m_wheelsPID.atSetpoint();
  }

  public double getEncoderVelocity() {
    //System.out.println(m_wheelsMotor.getEncoder().getVelocity());
    return m_wheelsMotor.getEncoder().getVelocity();
  }

}
