/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.shooterWheel.TraversoShooterWheelConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterWheel extends SubsystemBase {

  TraversoShooterWheelConstants constants = new TraversoShooterWheelConstants();

  private final WPI_TalonFX m_ShooterWheelMotor = ControllerFactory.createTalonFX(constants.kShooterWheelMotorPort);
  private final TalonEncoder m_ShooterWheelEncoder = new TalonEncoder(m_ShooterWheelMotor);

  private final PIDController ShooterWheelPID = new PIDController(constants.kP, constants.kI, constants.kD);

  public static double motorSpeed = 0.0;

  public ShooterWheel() {
    m_ShooterWheelEncoder.setDistancePerPulse(constants.kEncoderMetersPerPulse);
    m_ShooterWheelEncoder.reset();
    ShooterWheelPID.setTolerance(0, constants.kVelocityPIDTolerance);
    ShooterWheelPID.reset();
    ShooterWheelPID.setSetpoint(motorSpeed);
  }

  @Override
  public void periodic() {
    updatePID();
  }

  public void updatePID() {
    m_ShooterWheelMotor.set(ControlMode.PercentOutput, ShooterWheelPID.calculate(m_ShooterWheelEncoder.getRate()));
  }

  public void setSpeed(double newSpeed) {
    ShooterWheelPID.setSetpoint(newSpeed);
  }

  public void setBackOuttakeSpeed() {
    setSpeed(constants.kBackOuttakeSpeed);
  }

  public void setFrontOuttakeSpeed() {
    setSpeed(constants.kFrontOuttakeSpeed);
  }

  // TODO: Limelight integration
  /*
   * public void setFrontOuttakeFarSpeed() {
   * }
   * 
   * public void setBackOuttakeFarSpeed() {
   * }
   */

  public void stop() {
    m_ShooterWheelMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean reachedSetpoint() {
    return ShooterWheelPID.atSetpoint();
  }

}
