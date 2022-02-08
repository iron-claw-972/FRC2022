package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.IntakeConstants;

public class Intake {

  private final WPI_TalonFX m_motor = ControllerFactory.createTalonFX(IntakeConstants.kIntakeMotorPort);

  public void run(double pow) {
    m_motor.set(ControlMode.PercentOutput, pow);
  }
}
