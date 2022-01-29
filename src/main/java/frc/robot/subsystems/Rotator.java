package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.ControllerFactory;
import frc.robot.Constants.RotatorConstants;


public class Rotator {
    //TODO: check if whatever is connected to the sparkmax is brush/brushless
    private final CANSparkMax m_leftMotor = ControllerFactory.createSparkMAX(RotatorConstants.kLeftRotatorPort, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = ControllerFactory.createSparkMAX(RotatorConstants.kRightRotatorPort, MotorType.kBrushless);
    
    Encoder m_leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    Encoder m_rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);

    public Rotator() {
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);
    }

    public void run(double pow) {
        // runs but doesn't stop running even if it's at the design limit
        m_leftMotor.set(-pow);
        m_rightMotor.set(pow);
    }
}
