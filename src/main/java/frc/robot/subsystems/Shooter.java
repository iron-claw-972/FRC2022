/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX m_motor;
  private final TalonEncoder m_encoder;

  public PIDController m_shooterPID = new PIDController(Constants.shooter.kP, Constants.shooter.kI,
      Constants.shooter.kD);

  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      Constants.shooter.kFlywheelPlant,
      VecBuilder.fill(2.0), // How accurate we think our model is
      VecBuilder.fill(0.01), // How accurate we think our encoder
      // data is
      0.020);

  private final LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(
      Constants.shooter.kFlywheelPlant,
      VecBuilder.fill(Constants.shooter.kVelocityTolerance), // Velocity error tolerance
      VecBuilder.fill(12.0), // Control effort (voltage) tolerance
      0.020);

  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(Constants.shooter.kFlywheelPlant,
      m_controller, m_observer, 12.0, 0.020);

  private FlywheelSim m_flywheelSim;
  private TalonEncoderSim m_flywheelEncoderSim;

  private boolean m_enabled = false;
  private double m_motorSpeed = 0.0;

  private double m_lastTimestamp;

  public Shooter() {
    this(ControllerFactory.createTalonFX(
        Constants.shooter.kCargoShooterMotorPort,
        Constants.shooter.kSupplyCurrentLimit,
        Constants.shooter.kSupplyTriggerThreshold,
        Constants.shooter.kSupplyTriggerDuration,
        Constants.shooter.kNeutral));
  }

  public Shooter(WPI_TalonFX motor) {
    m_motor = motor;

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_encoder = new TalonEncoder(motor);

    if (RobotBase.isSimulation()) {
      m_flywheelSim = new FlywheelSim(
          Constants.shooter.kFlywheelPlant,
          Constants.shooter.kFlywheelGearbox,
          Constants.shooter.kGearRatio);

      m_flywheelEncoderSim = new TalonEncoderSim(m_encoder);
    }

    m_encoder.setDistancePerPulse(Constants.shooter.kDistancePerPulse);
    m_encoder.reset();
    m_shooterPID.setTolerance(Constants.shooter.kVelocityTolerance);
    m_shooterPID.reset();
    // cargoShooterPID.setSetpoint(motorSpeed);

    m_loop.reset(VecBuilder.fill(m_encoder.getRate()));


    m_lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    // if (m_enabled) {

      m_shooterPID.setSetpoint(m_motorSpeed);
      m_shooterPID.calculate(getVelocity());
      // setVoltage(m_shooterPID.calculate(getVelocity()) + (0.00128 * m_motorSpeed + ((m_motorSpeed > 0 ? 1 : -1) * Constants.shooter.kForward)));
      // setVoltage(m_shooterPID.calculate(getVelocity()) + (0.0013 * m_motorSpeed - SmartDashboard.getNumber("Shooter FF", Constants.shooter.kForward)));
      // setVoltage(m_shooterPID.calculate(getVelocity()) +
      // SmartDashboard.getNumber("Shooter FF", Constants.shooter.kForward) *
      // m_motorSpeed);
      // setVoltage(SmartDashboard.getNumber("Shooter FF", Constants.shooter.kForward));
      // setVoltage(m_shooterPID.calculate(getVelocity()) + Constants.shooter.kForward * m_motorSpeed);

      m_loop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(m_motorSpeed)));

      m_loop.correct(VecBuilder.fill(m_encoder.getRate())); // Encoder gives rad/s

      double timestamp = Timer.getFPGATimestamp();
      // Update our LQR to generate new voltage commands and use the voltages to predict the next
      // state with out Kalman filter.
      m_loop.predict(timestamp - m_lastTimestamp);

      // System.out.println(m_encoder.getRate());

      // Send the new calculated voltage to the motors.
      // voltage = duty cycle * battery voltage, so
      // duty cycle = voltage / battery voltage
      double nextVoltage = m_loop.getU(0);
      if (RobotBase.isReal()) {
        nextVoltage += SmartDashboard.getNumber("Shooter kS", Constants.shooter.kS) * Math.signum(m_motorSpeed);
      }
      m_motor.setVoltage(m_motorSpeed == 0 ? 0 : nextVoltage);
      m_lastTimestamp = timestamp;
    // }
  }

  @Override
  public void simulationPeriodic() {
    m_flywheelSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
    m_flywheelSim.update(0.020);

    m_flywheelEncoderSim.setRate(m_flywheelSim.getAngularVelocityRadPerSec());
    // SmartDashboard.putNumber("Velocity (RPM)",
    // m_flywheelSim.getAngularVelocityRPM());
  }

  public void setOutput(double motorPower) {
    // m_motor.set(ControlMode.PercentOutput,
    //     MathUtil.clamp(motorPower, -Constants.shooter.kMotorClamp, Constants.shooter.kMotorClamp));
  }

  public void setSpeed(double newSpeed) {
    m_motorSpeed = newSpeed;
    // m_motorSpeed = newSpeed;
    m_shooterPID.setSetpoint(m_motorSpeed);
  }

  public void setStop() {
    setSpeed(0);
  }

  public void enable() {
    m_enabled = true;
  }

  public void disable() {
    m_enabled = false;
    setSpeed(0.0);
    setVoltage(0);
  }

  public boolean reachedSetpoint() {
    return m_shooterPID.atSetpoint();
    // System.out.println(m_motorSpeed);
    // return Math.abs(getVelocity() - m_motorSpeed) <= Constants.shooter.kVelocityTolerance;
  }

  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
  }

  public double getVelocity() {
    // return m_encoder.getRate();
    return Units.radiansPerSecondToRotationsPerMinute(m_encoder.getRate());
  }

  public boolean isEnabled() {
    return m_enabled;
  }
  
}
