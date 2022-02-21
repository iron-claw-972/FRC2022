/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class CargoShooter extends SubsystemBase {

  TraversoCargoShooterConstants constants = new TraversoCargoShooterConstants();

  private final WPI_TalonFX m_CargoShooterMotor = ControllerFactory.createTalonFX(constants.kCargoShooterMotorPort , constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast);
  private final TalonEncoder m_CargoShooterEncoder = new TalonEncoder(m_CargoShooterMotor);

  private final PIDController CargoShooterPID = new PIDController(constants.kP, constants.kI, constants.kD);
  


  private boolean enabled = false;
  private double motorSpeed = 0.0;

  public CargoShooter() {
    m_CargoShooterEncoder.setDistancePerPulse(constants.kEncoderMetersPerPulse);
    m_CargoShooterEncoder.reset();
    CargoShooterPID.setTolerance(constants.kVelocityPIDTolerance);
    CargoShooterPID.reset();
    CargoShooterPID.setSetpoint(motorSpeed);
  }

  @Override
  public void periodic() {
    if (enabled){
      setOutput(CargoShooterPID.calculate(m_CargoShooterEncoder.getRate()));
    }
  }

  public void setOutput(double motorPower) {
    m_CargoShooterMotor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  public void setSpeed(double newSpeed) {
    CargoShooterPID.setSetpoint(newSpeed);
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
    return CargoShooterPID.atSetpoint();
  }

  // public double getVelocity(double distance, boolean isFront) {
  // }

}
