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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoShooter extends SubsystemBase {

  TraversoCargoShooterConstants constants = new TraversoCargoShooterConstants();

  private final WPI_TalonFX m_CargoShooterMotor = ControllerFactory.createTalonFX(constants.kCargoShooterMotorPort , constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast);
  private final TalonEncoder m_CargoShooterEncoder = new TalonEncoder(m_CargoShooterMotor);

  public PIDController cargoShooterPID = new PIDController(constants.kP, constants.kI, constants.kD);

  private boolean enabled = false;
  private double motorSpeed = 0.0;

  public CargoShooter() {
    enable();
    m_CargoShooterEncoder.setDistancePerPulse(constants.kDistancePerPulse);
    m_CargoShooterEncoder.reset();
    cargoShooterPID.setTolerance(constants.kVelocityPIDTolerance);
    cargoShooterPID.reset();
    cargoShooterPID.setSetpoint(motorSpeed);
  }

  @Override
  public void periodic() {
    if (enabled){
      cargoShooterPID.setSetpoint(motorSpeed);
      setVoltage(cargoShooterPID.calculate(getVelocity()) + constants.kForward * motorSpeed);
    }
  }

  public void setOutput(double motorPower) {
    m_CargoShooterMotor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  public void setSpeed(double newSpeed) {
    motorSpeed = newSpeed;
  }

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
    return cargoShooterPID.atSetpoint();
  }

  public void setVoltage(double volts){
    m_CargoShooterMotor.setVoltage(volts);
  }

  public double getVelocity(){
    return m_CargoShooterEncoder.getRate();
  }

  public boolean isEnabled() {
    return enabled;
  }

}
