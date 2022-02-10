package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterWheels extends SubsystemBase {

  private final CANSparkMax m_wheelsMotor = new CANSparkMax(8, MotorType.kBrushless);
  
  private final CANSparkMax m_wheelsMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final PIDController m_wheelsPID = new PIDController(ShooterConstants.kBottomMotorP,
      ShooterConstants.kBottomMotorI, ShooterConstants.kBottomMotorD);

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

  public void updatePID() {
    System.out.println("Speed: " + getEncoderVelocity());
    double pow = m_wheelsPID.calculate(getEncoderVelocity(), motorSpeed);
    System.out.println("goal: " + motorSpeed);
    System.out.println("Power: " + ((12.0 / 5676.0)*motorSpeed + pow));
    m_wheelsMotor.set((12.0 / 5676.0)*motorSpeed + pow);
  }

  public void setSpeed(double speed) {
    PIDenabled = true;
    motorSpeed = speed;
    //m_wheelsMotor.set(0.5);
  }

  public void setIntakeSpeed() {
    setSpeed(ShooterConstants.kShooterBeltIntakeSpeed);
  }

  public void setOutakeSpeed() {
    setSpeed(ShooterConstants.kShooterBeltOutakeSpeed);
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
    return m_wheelsMotor.getEncoder().getVelocity();
  }

}
