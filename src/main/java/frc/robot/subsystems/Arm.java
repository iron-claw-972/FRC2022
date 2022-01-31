package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;

public class Arm {
    
    private final WPI_TalonFX m_leftMotor  = ControllerFactory.createTalonFX( ArmConstants.kLeftMotorPort);
    private final WPI_TalonFX m_rightMotor = ControllerFactory.createTalonFX(ArmConstants.kRightMotorPort);

    private final PhoenixMotorControllerGroup m_motors = new PhoenixMotorControllerGroup(m_leftMotor, m_rightMotor);

// arm encoder
  private final TalonEncoder m_leftEncoder = new TalonEncoder(m_leftMotor, DriveConstants.kLeftEncoderReversed);
  private final TalonEncoder m_rightEncoder = new TalonEncoder(m_rightMotor, DriveConstants.kRightEncoderReversed);

    
}
