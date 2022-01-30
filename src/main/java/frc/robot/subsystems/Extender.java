package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ExtenderConstants;


public class Extender {
    private final WPI_TalonFX m_leftMotor = ControllerFactory.createTalonFX(ExtenderConstants.kLeftExtenderPort);
    private final WPI_TalonFX m_rightMotor = ControllerFactory.createTalonFX(ExtenderConstants.kRightExtenderPort);
    
    private final PIDController extenderPID = new PIDController(1, 0, 0);

    public Extender() {
        m_rightMotor.follow(m_leftMotor);

        // so that the motors spin in the same direction
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);

        // so that encoder values aren't negative
        m_rightMotor.setSensorPhase(true);
        m_leftMotor.setSensorPhase(false);

        // the lowest tick limit is 0, and must be checked every 10 milliseconds
        m_leftMotor.configReverseSoftLimitThreshold(0, 10);
        m_rightMotor.configReverseSoftLimitThreshold(0, 10);

        // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
        m_leftMotor.configForwardSoftLimitThreshold(ExtenderConstants.kExtenderMaxArmLength / ExtenderConstants.kExtenderTickMultiple, 10);
        m_rightMotor.configForwardSoftLimitThreshold(ExtenderConstants.kExtenderMaxArmLength / ExtenderConstants.kExtenderTickMultiple, 10);

        // every time the robot is started, it MUST start at maximum compression in order to maintain consistency
        // TODO: Make this better.
        m_leftMotor.setSelectedSensorPosition(0.0);
        m_rightMotor.setSelectedSensorPosition(0.0);
    }

    // called in RobotContainer by configureButtonBindings()
    public void extendClimberArm(double setpoint) {
        m_leftMotor.set(extenderPID.calculate(m_rightMotor.getSelectedSensorPosition() * ExtenderConstants.kExtenderTickMultiple, setpoint));
        // a pop-up in shuffleboard that allows you to see how much the arm extended in feet
        SmartDashboard.putNumber("Current Extension (Inches)", m_rightMotor.getSelectedSensorPosition() * ExtenderConstants.kExtenderTickMultiple);
    }
} 
