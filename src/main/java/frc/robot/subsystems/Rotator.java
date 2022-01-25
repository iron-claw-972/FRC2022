package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.ControllerFactory;
import frc.robot.Constants.RotatorConstants;

public class Rotator {
    private final CANSparkMax m_lmotor = ControllerFactory.createSparkMAX(RotatorConstants.kLeftRotatorPort, MotorType.kBrushless);
    private final CANSparkMax m_rmotor = ControllerFactory.createSparkMAX(RotatorConstants.kRightRotatorPort, MotorType.kBrushless);

    public void runLeft(double pow) {
        m_lmotor.set(pow);
    }

    public void runRight(double pow) {
        m_rmotor.set(pow);
    }
}
