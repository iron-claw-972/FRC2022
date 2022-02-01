package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ExtenderConstants;


public class Extender {
    final WPI_TalonFX m_motor;
    
    private final PIDController extenderPID = new PIDController(1, 0, 0);

    public Extender(int port, boolean left) {
        m_motor = ControllerFactory.createTalonFX(port);

        if(left == true) {
            // so that the arm doesn't spin in an opposing direction
            port*=-1;
            // so that encoder values aren't negative
            m_motor.setSensorPhase(true);
        }

        // the lowest tick limit is 0, and must be checked every 10 milliseconds
        m_motor.configReverseSoftLimitThreshold(0, 10);
        m_motor.configReverseSoftLimitThreshold(0, 10);

        // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
        m_motor.configForwardSoftLimitThreshold(ExtenderConstants.kExtenderMaxArmLength / ExtenderConstants.kExtenderTickMultiple, 10);

        // every time the robot is started, it MUST start at maximum compression in order to maintain consistency
        // TODO: Make this better.
        m_motor.setSelectedSensorPosition(0.0);
    }

    // called in RobotContainer by configureButtonBindings()
    public void extendClimberArm(double setpoint) {
        m_motor.set(extenderPID.calculate(m_motor.getSelectedSensorPosition() * ExtenderConstants.kExtenderTickMultiple, setpoint));

        // if it's within a certain threshold, return true
        reachedPoint(setpoint);

        // a pop-up in shuffleboard that allows you to see how much the arm extended in feet
        SmartDashboard.putNumber("Current Extension (Inches)", m_motor.getSelectedSensorPosition() * ExtenderConstants.kExtenderTickMultiple);
    }

    public boolean reachedPoint(double max) {
        // if the current tick value (when negative) is greater than 10 ticks from the setpoint, return true
        if(m_motor.getSelectedSensorPosition() <= (ExtenderConstants.kExtenderMaxArmLength / ExtenderConstants.kExtenderTickMultiple) + 10 && max < 0.0) {
            return true;
        }
        // if the current tick value (when positive) is greater than 10 ticks from the setpoint, return true
        else if(m_motor.getSelectedSensorPosition() >= (ExtenderConstants.kExtenderMaxArmLength / ExtenderConstants.kExtenderTickMultiple) - 10 && max > 0.0) {
            return true;
        }
        //otherwise, return false
        return false;
    }
} 
