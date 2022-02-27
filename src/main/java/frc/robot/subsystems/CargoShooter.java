/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoShooter extends SubsystemBase {

  TraversoCargoShooterConstants constants = new TraversoCargoShooterConstants();

  private final WPI_TalonFX m_cargoShooterMotor = ControllerFactory.createTalonFX(constants.kCargoShooterMotorPort , constants.kSupplyCurrentLimit, constants.kSupplyTriggerThreshold, constants.kSupplyTriggerDuration, constants.kCoast);
  private final TalonEncoder m_cargoShooterEncoder = new TalonEncoder(m_cargoShooterMotor);

  public PIDController cargoShooterPID = new PIDController(constants.kP, constants.kI, constants.kD);

  private FlywheelSim m_flywheelSim;
  private TalonEncoderSim m_flywheelEncoderSim;

  private boolean enabled = false;
  private double motorSpeed = 0.0;

  public CargoShooter() {
    m_cargoShooterEncoder.setDistancePerPulse(constants.kDistancePerPulse);
    m_cargoShooterEncoder.reset();
    cargoShooterPID.setTolerance(constants.kVelocityPIDTolerance);
    cargoShooterPID.reset();
    cargoShooterPID.setSetpoint(motorSpeed);

    if (RobotBase.isSimulation()) {
      m_flywheelSim = new FlywheelSim(
        constants.kFlywheelPlant,
        constants.kFlywheelGearbox,
        constants.kGearRatio
      );

      m_flywheelEncoderSim = new TalonEncoderSim(m_cargoShooterEncoder);
    }
  }

  @Override
  public void periodic() {
    if (enabled){
      cargoShooterPID.setSetpoint(motorSpeed);
      setVoltage(cargoShooterPID.calculate(getVelocity()) + constants.kForward * motorSpeed);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_flywheelSim.setInput(m_cargoShooterMotor.get() * RobotController.getBatteryVoltage());
    m_flywheelSim.update(0.020);

    m_flywheelEncoderSim.setRate(m_flywheelSim.getAngularVelocityRPM() / 60);
    // SmartDashboard.putNumber("Velocity (RPM)", m_flywheelSim.getAngularVelocityRPM());
  }

  // public void setOutput(double motorPower) {
  //   m_cargoShooterMotor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  // }

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
    setVoltage(0);
  }

  public boolean reachedSetpoint() {
    return cargoShooterPID.atSetpoint();
  }

  public void setVoltage(double volts){
    m_cargoShooterMotor.setVoltage(volts);
  }

  public double getVelocity(){
    return m_cargoShooterEncoder.getRate();
  }

  public boolean isEnabled() {
    return enabled;
  }

}
