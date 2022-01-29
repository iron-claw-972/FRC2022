package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ExtenderConstants;

public class Extender {
    //TODO: check if the extender motors are talonfx motors
    private final WPI_TalonFX m_lmotor = ControllerFactory.createTalonFX(ExtenderConstants.kLeftExtenderPort);
    private final WPI_TalonFX m_rmotor = ControllerFactory.createTalonFX(ExtenderConstants.kRightExtenderPort);

    public void run(double pow) {
        m_lmotor.set(pow);
        m_rmotor.set(-pow);
    }
}
