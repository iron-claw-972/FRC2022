package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import ctre_shims.TalonEncoder;

import frc.robot.ControllerFactory;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ExtenderConstants;


public class Extender {
    //TODO: check if the extender motors are talonfx motors
    private final WPI_TalonFX m_leftMotor = ControllerFactory.createTalonFX(ExtenderConstants.kLeftExtenderPort);
    private final WPI_TalonFX m_rightMotor = ControllerFactory.createTalonFX(ExtenderConstants.kRightExtenderPort);

    private final TalonEncoder m_leftEncoder = new TalonEncoder(m_leftMotor, DriveConstants.kLeftEncoderReversed);
    private final TalonEncoder m_rightEncoder = new TalonEncoder(m_rightMotor, DriveConstants.kRightEncoderReversed);

    private boolean disabler = false;

    public void run(double pow) {
        m_leftMotor.set(pow);
        m_rightMotor.set(-pow);
    }
}
