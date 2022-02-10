/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterBelt extends SubsystemBase {

  TraversoBeltConstants constants = new TraversoBeltConstants();

  private final WPI_TalonFX m_ShooterBeltMotor = ControllerFactory.createTalonFX(constants.kShooterBeltMotorPort);
  private final TalonEncoder m_ShooterBeltEncoder = new TalonEncoder(m_ShooterBeltMotor);

  private final PIDController ShooterBeltPID = new PIDController(constants.kShooterBeltP, constants.kShooterBeltI, constants.kShooterBeltD);

  public static double motorSpeed = 0.0;

  public ShooterBelt() {
    m_ShooterBeltEncoder.setDistancePerPulse(constants.kEncoderMetersPerPulse);
    m_ShooterBeltEncoder.reset();
    ShooterBeltPID.reset();
    ShooterBeltPID.setTolerance(constants.kShooterBeltVelocityPIDTolerance);
    ShooterBeltPID.setSetpoint(motorSpeed);
  }

  @Override
  public void periodic() {
    updatePID();
  }

  public void updatePID() {
    m_ShooterBeltMotor.set(ControlMode.PercentOutput, ShooterBeltPID.calculate(m_ShooterBeltEncoder.getRate()));
  }

  public void setSpeed(double newSpeed) {
    ShooterBeltPID.setSetpoint(newSpeed);
  }

  public void setIntakeSpeed() {
    setSpeed(constants.kIntakeSpeed);
  }

  public void setOuttakeSpeed() {
    setSpeed(constants.kOuttakeSpeed);
  }

  public void stop() {
    m_ShooterBeltMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean reachedSetpoint(double targetSpeed) {
    return ShooterBeltPID.atSetpoint();
  }

}