/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.constants.shooterWheel.TraversoShooterWheelConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterWheel extends SubsystemBase {

  TraversoShooterWheelConstants constants = new TraversoShooterWheelConstants();

  private final WPI_TalonFX m_ShooterWheelMotor = ControllerFactory.createTalonFX(constants.kShooterWheelMotorPort);
  private final TalonEncoder m_ShooterWheelEncoder = new TalonEncoder(m_ShooterWheelMotor);

  private final PIDController ShooterWheelPID = new PIDController(constants.kShooterWheelP,
  constants.kShooterWheelI, constants.kShooterWheelD);

  public static double motorSpeed = 1.0;

  public ShooterWheel() {
    m_ShooterWheelEncoder.setDistancePerPulse(constants.kEncoderMetersPerPulse);
    m_ShooterWheelEncoder.reset();
  }

  public void updatePID() {
    m_ShooterWheelMotor.set(ControlMode.PercentOutput, ShooterWheelPID.calculate(motorSpeed));
  }

  public void setSpeed(double newSpeed) {
    motorSpeed = newSpeed;
  }

  public void setBackOuttakeSpeed() {
    motorSpeed = constants.kBackOuttakeSpeed;
  }

  public void setFrontOuttakeSpeed() {
    motorSpeed = constants.kFrontOuttakeSpeed;
  }

  // TODO: Limelight integration
  /*
   * public void setFrontOuttakeFarSpeed() {
   * motorSpeed = 1.0;
   * }
   * 
   * public void setBackOuttakeFarSpeed() {
   * motorSpeed = 1.0;
   * }
   */

  public void stop() {
    m_ShooterWheelMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean reachedSetpoint(double targetSpeed) {
    return (m_ShooterWheelEncoder.getRate() < targetSpeed + constants.kShooterWheelVelocityPIDTolerance &&
            m_ShooterWheelEncoder.getRate() > targetSpeed - constants.kShooterWheelVelocityPIDTolerance);
  }

}
