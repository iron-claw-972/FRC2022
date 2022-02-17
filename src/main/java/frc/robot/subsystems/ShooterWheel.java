/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.shooterWheel.TraversoShooterWheelConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterWheel extends SubsystemBase {

  TraversoShooterWheelConstants constants = new TraversoShooterWheelConstants();

  private final WPI_TalonFX m_ShooterWheelMotor = ControllerFactory.createTalonFX(constants.kShooterWheelMotorPort);
  private final TalonEncoder m_ShooterWheelEncoder = new TalonEncoder(m_ShooterWheelMotor);

  private final PIDController m_ShooterWheelPID = new PIDController(constants.kP, constants.kI, constants.kD);
  
  private boolean enabled = false;
  private double motorSpeed = 0.0;
  private double clampedPower;

  public ShooterWheel() {
    m_ShooterWheelEncoder.setDistancePerPulse(constants.kEncoderMetersPerPulse);
    m_ShooterWheelEncoder.reset();
    m_ShooterWheelPID.setTolerance(constants.kVelocityPIDTolerance);
    m_ShooterWheelPID.reset();
    m_ShooterWheelPID.setSetpoint(motorSpeed);
  }

  @Override
  public void periodic() {
    if (enabled){
      setOutput(m_ShooterWheelPID.calculate(m_ShooterWheelEncoder.getRate()));
    }
  }

  public void setOutput(double motorPower) {
    clampedPower = MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp);
    // (constants.kVolts / constants.kRPM)*motorSpeed + ShooterWheelPID.calculate(m_shooterMotor.getEncoder().getVelocity(), motorSpeed)
    m_ShooterWheelMotor.setVoltage((constants.kVolts / constants.kRPM)*clampedPower + m_ShooterWheelPID.calculate(m_ShooterWheelEncoder.getRate(), clampedPower));
  }

  public void setSpeed(double newSpeed) {
    m_ShooterWheelPID.setSetpoint(newSpeed);
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

  public void setStop() {
    setSpeed(0);
  }

  public void enable() {
    enabled=true;
  }

  public void disable() {
    enabled=false;
    setOutput(0);
  }

  public boolean reachedSetpoint() {
    return m_ShooterWheelPID.atSetpoint();
  }



}
