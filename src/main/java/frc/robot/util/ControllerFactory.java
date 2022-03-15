package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class ControllerFactory {

  private static int talonSRXDefaultContinuousLimit = 38;
  private static int talonSRXDefaultPeakLimit = 45;
  private static int talonSRXDefaultPeakDuration = 125;


  /*
  * Talon Sator / Supply Limits explanation 
  * Supply current is current that’s being drawn at the input bus voltage. Stator current is current that’s being drawn by the motor.
  * Supply limiting (supported by Talon SRX and FX) is useful for preventing breakers from tripping in the PDP.
  * Stator limiting (supported by Talon FX) is useful for limiting acceleration/heat.
  */

  public static final boolean talonFXStatorLimitEnable = false; // enabled?
  public static final double talonFXStatorCurrentLimit = 100; // Limit(amp)
  public static final double talonFXStatorTriggerThreshold = 100; // Trigger Threshold(amp)
  public static final double talonFXStatorTriggerDuration = 0; // Trigger Threshold Time(s)

  public static final boolean talonFXSupplyLimitEnable = false;  // enabled?
  public static final double talonFXSupplyCurrentLimit = 40;     // Limit(amp), usual current to hold after trigger hit
  public static final double talonFXSupplyTriggerThreshold = 55; // Trigger Threshold(amp), amps to activate trigger 
  public static final double talonFXSupplyTriggerDuration = 3; // Trigger Threshold Time(s), how long after trigger before reducing

  private static int sparkMAXDefaultCurrentLimit = 60;

  private static double voltageCompensation = Constants.kMaxVoltage; 

  /**
   * Create a TalonSRX with current limiting enabled, using parameters
   * 
   * @param id the ID of the TalonSRX
   * @param continuousCurrentLimit the continuous current limit to set in amps (A)
   * @param peakCurrentLimit the peak current limit to set in amps (A)
   * @param peakCurrentDuration the peak current limit duration to set in milliseconds (ms)
   * 
   * @return a fully configured TalonSRX object
   */
  
  public static WPI_TalonSRX createTalonSRX(int id, int continuousCurrentLimit, int peakCurrentLimit, int peakCurrentDuration) {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.continuousCurrentLimit = continuousCurrentLimit;
    config.peakCurrentLimit = peakCurrentLimit;
    config.peakCurrentDuration = peakCurrentDuration;
    config.voltageCompSaturation = voltageCompensation;

    WPI_TalonSRX talon = new WPI_TalonSRX(id);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableCurrentLimit(true);
    talon.enableVoltageCompensation(true);
    talon.setNeutralMode(NeutralMode.Brake);

    return talon;
  }

  /**
   * Create a TalonSRX using default current limits, with the option to disable current limiting entirely
   * 
   * @param id the ID of the TalonSRX
   * @param useDefaultLimits whether or not to enable the default limits
   * @return a fully configured TalonSRX object 
   */
  public static WPI_TalonSRX createTalonSRX(int id) {
    WPI_TalonSRX talon = createTalonSRX(id, talonSRXDefaultContinuousLimit, talonSRXDefaultPeakLimit, talonSRXDefaultPeakDuration);
    talon.enableCurrentLimit(true);

    return talon;
  }

  /**
   * Create a CANSparkMax with current limiting enabled
   * 
   * @param id the ID of the Spark MAX
   * @param motortype the type of motor the Spark MAX is connected to 
   * @param stallLimit the current limit to set at stall
   * 
   * @return a fully configured CANSparkMAX
   */
  public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit) {
    CANSparkMax sparkMAX = new CANSparkMax(id, motortype);
    sparkMAX.restoreFactoryDefaults();
    sparkMAX.enableVoltageCompensation(voltageCompensation);
    sparkMAX.setSmartCurrentLimit(stallLimit);
    sparkMAX.setIdleMode(IdleMode.kBrake);

    sparkMAX.burnFlash();
    return sparkMAX;
  }

  /**
  * Create a CANSparkMax with default current limiting enabled
  * 
  * @param id the ID of the Spark MAX
  * @param motortype the type of motor the Spark MAX is connected to
  * 
  * @return a fully configured CANSparkMAX
  */
  public static CANSparkMax createSparkMAX(int id, MotorType motortype) {
    return createSparkMAX(id, motortype, sparkMAXDefaultCurrentLimit);
  }

  /**
  * Create a configured TalonFX 
  * https://motors.vex.com/vexpro-motors/falcon
  * 
  * @param id the ID of the motor
  * 
  * @return a fully configured TalonFX
  */
  public static WPI_TalonFX createTalonFX(int id) {

    if (id == -1) return null;

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(
     // enabled                  | Limit(amp)               | Trigger Threshold(amp) |       Trigger Threshold Time(s)  */
        talonFXStatorLimitEnable, talonFXStatorCurrentLimit, talonFXStatorTriggerThreshold, talonFXStatorTriggerDuration);
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        talonFXSupplyLimitEnable, talonFXSupplyCurrentLimit, talonFXSupplyTriggerThreshold, talonFXSupplyTriggerDuration);
    
    config.voltageCompSaturation = Constants.kMaxVoltage;

    WPI_TalonFX talon = new WPI_TalonFX(id);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableVoltageCompensation(false);
    talon.setNeutralMode(NeutralMode.Coast);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    return talon;

  }
  /**
  * Create a configured TalonFX 
  * https://motors.vex.com/vexpro-motors/falcon
  * 
  * @param id the ID of the motor
  * @param supplyCurrentLimit the regular current to return to after the trigger
  * @param supplyTriggerThreshold The current at which the trigger will activate
  * @param supplyTriggerDuration The amount of time the current must be above the trigger current to reduce current
  * @param neutralMode Whether the motor is in coast or brake mode
  *
  * @return a fully configured TalonFX
  */
  public static WPI_TalonFX createTalonFX(int id, double supplyCurrentLimit, double supplyTriggerThreshold, double supplyTriggerDuration , NeutralMode neutralMode) {

    if (id == -1) return null;

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        true, supplyCurrentLimit, supplyTriggerThreshold, supplyTriggerDuration);
    
    config.voltageCompSaturation = Constants.kMaxVoltage;

    WPI_TalonFX talon = new WPI_TalonFX(id);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableVoltageCompensation(false);
    talon.setNeutralMode(neutralMode);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    return talon;

  }
}