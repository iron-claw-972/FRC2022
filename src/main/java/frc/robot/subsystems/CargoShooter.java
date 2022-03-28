/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoShooter extends SubsystemBase {
  private final WPI_TalonFX m_motor = ControllerFactory.createTalonFX(
    Constants.shooter.kCargoShooterMotorPort,
    Constants.shooter.kSupplyCurrentLimit,
    Constants.shooter.kSupplyTriggerThreshold,
    Constants.shooter.kSupplyTriggerDuration,
    Constants.shooter.kNeutral
  );
  private final TalonEncoder m_encoder = new TalonEncoder(m_motor);

  public PIDController m_shooterPID = new PIDController(Constants.shooter.kP, Constants.shooter.kI, Constants.shooter.kD);

  private FlywheelSim m_flywheelSim;
  private TalonEncoderSim m_flywheelEncoderSim;

  private boolean m_enabled = false;
  private double m_motorSpeed = 0.0;

  public CargoShooter() {
    m_encoder.setDistancePerPulse(Constants.shooter.kDistancePerPulse);
    m_encoder.reset();
    m_shooterPID.setTolerance(Constants.shooter.kVelocityPIDTolerance);
    m_shooterPID.reset();
    // cargoShooterPID.setSetpoint(motorSpeed);

    if (RobotBase.isSimulation()) {
      m_flywheelSim = new FlywheelSim(
        Constants.shooter.kFlywheelPlant,
        Constants.shooter.kFlywheelGearbox,
        Constants.shooter.kGearRatio
      );

      m_flywheelEncoderSim = new TalonEncoderSim(m_encoder);
    }
  }

  @Override
  public void periodic() {
    if (m_enabled){
      
      m_shooterPID.setSetpoint(m_motorSpeed);
      setVoltage(m_shooterPID.calculate(getVelocity()) + SmartDashboard.getNumber("Shooter FF", Constants.shooter.kForward) * m_motorSpeed);
    ;
    }
  }

  @Override
  public void simulationPeriodic() {
    m_flywheelSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
    m_flywheelSim.update(0.020);

    m_flywheelEncoderSim.setRate(m_flywheelSim.getAngularVelocityRPM() / 60);
    // SmartDashboard.putNumber("Velocity (RPM)", m_flywheelSim.getAngularVelocityRPM());
  }

  public void setOutput(double motorPower) {
    m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -Constants.shooter.kMotorClamp, Constants.shooter.kMotorClamp));
  }

  public void setSpeed(double newSpeed) {
    m_motorSpeed = newSpeed;
    m_shooterPID.setSetpoint(m_motorSpeed);
  }

  public void setStop() {
    setSpeed(0);
  }

  public void enable() {
    m_enabled=true;
  }

  public void disable() {
    m_enabled=false;
    setVoltage(0);
  }

  public boolean reachedSetpoint() {
    return m_shooterPID.atSetpoint();
  }

  public void setVoltage(double volts){
    m_motor.setVoltage(volts);
  }

  public double getVelocity(){
    return m_encoder.getRate();
  }

  public boolean isEnabled() {
    return m_enabled;
  }

}
