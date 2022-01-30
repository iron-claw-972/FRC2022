package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControllerFactory;
import frc.robot.Constants.RotatorConstants;


public class Rotator {
    private final WPI_TalonFX m_leftMotor = ControllerFactory.createTalonFX(RotatorConstants.kLeftRotatorPort);
    private final WPI_TalonFX m_rightMotor = ControllerFactory.createTalonFX(RotatorConstants.kRightRotatorPort);

    public Rotator() {
        m_rightMotor.follow(m_leftMotor);

        // so that the motors spin in the same direction
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);

        // so that encoder values aren't negative
        m_rightMotor.setSensorPhase(true);
        m_leftMotor.setSensorPhase(false);

        // converts the minimum angle of the arm and makes that the minimum tick limit, it's checked every 10 milliseconds
        m_leftMotor.configReverseSoftLimitThreshold(-RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple, 10);
        m_rightMotor.configReverseSoftLimitThreshold(-RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple, 10);

        // converts the maximum angle of the arm to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
        m_leftMotor.configForwardSoftLimitThreshold(RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple, 10);
        m_rightMotor.configForwardSoftLimitThreshold(RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple, 10);

        // every time the robot is started, it MUST start at a 90 degree angle in order to maintain consistency
        // TODO: Make this better.
        m_leftMotor.setSelectedSensorPosition(0.0);
        m_rightMotor.setSelectedSensorPosition(0.0);
    }
    
    // called in RobotContainer by configureButtonBindings()
    public void rotateArm(double pow) {
        m_rightMotor.set(pow);
        // a pop-up in shuffleboard that allows you to see how much the arm angles in degrees
        SmartDashboard.putNumber("Current Angle (Degrees)", m_rightMotor.getSelectedSensorPosition() * RotatorConstants.kRotatorTickMultiple);
    }
}
