package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final DutyCycleEncoder dce;
  private final WPI_TalonFX m_motor;
  private double currentTickVal;

  private double setpoint = 0;

  private PIDController armPID = new PIDController(0.02, 0, 0);

  public Arm(boolean left) {

    // if the arm is left, the encoder value is inverted && the 
    if (left) {
      dce = new DutyCycleEncoder(ArmConstants.kArmLeftEncoder);
      m_motor = ControllerFactory.createTalonFX(ArmConstants.kArmLeftMotor);
      currentTickVal = dce.get() * -1;
    }
    // otherwise, use the normal encoder value and set the motorports to the right
    else {
      dce = new DutyCycleEncoder(ArmConstants.kArmRightEncoder);
      m_motor = ControllerFactory.createTalonFX(ArmConstants.kArmRightMotor);
      currentTickVal = dce.get();
    }
  }

  public boolean reachedSetpoint() {
    // if the current tick position is within the setpoint's range (setpoint +- 10), return true, otherwise return false
    return(currentTickVal) >= (setpoint / ArmConstants.kArmDegreeMultiple) - ArmConstants.kArmTolerance
    && currentTickVal <= (setpoint / ArmConstants.kArmDegreeMultiple) + ArmConstants.kArmTolerance;
  }

  // called in RobotContainer by button binds
  public void set(double distance){
    setpoint = distance;
  }


  @Override
  public void periodic(){
    if (reachedSetpoint() == false) {
      // sets the motor to go to a setpoint
      // the setpoint is tick value
      m_motor.set(armPID.calculate(currentTickVal, setpoint));

      // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
      SmartDashboard.putNumber("Current Angle (Degrees)", dce.get() * ArmConstants.kArmDegreeMultiple);
    }
  }
}