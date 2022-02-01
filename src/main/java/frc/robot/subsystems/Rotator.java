package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControllerFactory;
import frc.robot.Constants.RotatorConstants;


public class Rotator {
    private final WPI_TalonFX m_motor;

    private final PIDController rotatorPID = new PIDController(1, 0, 0);

    public Rotator(int port, boolean left) {
        m_motor = ControllerFactory.createTalonFX(port);

        if(left == true) {
            // so that the arm doesn't spin in an opposing direction
            port*=-1;
            // so that encoder values aren't negative
            m_motor.setSensorPhase(true);
            m_motor.setInverted(true);
        }

        // converts the minimum angle of the arm and makes that the minimum tick limit, it's checked every 10 milliseconds
        m_motor.configReverseSoftLimitThreshold(-RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple, 10);

        // converts the maximum angle of the arm to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
        m_motor.configForwardSoftLimitThreshold(RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple, 10);

        // every time the robot is started, it MUST start at a 90 degree angle in order to maintain consistency
        // TODO: Make this better.
        m_motor.setSelectedSensorPosition(0.0);
    }
    
    // called in RobotContainer by configureButtonBindings()
    public void rotateArm(double setpoint) {
        m_motor.set(rotatorPID.calculate(m_motor.getSelectedSensorPosition() * RotatorConstants.kRotatorTickMultiple, setpoint));

        // if it's within a certain threshold, return true
        reachedPoint(setpoint);


        // a pop-up in shuffleboard that allows you to see how much the arm angles in degrees
        SmartDashboard.putNumber("Current Angle (Degrees)", m_motor.getSelectedSensorPosition() * RotatorConstants.kRotatorTickMultiple);
    }

    public boolean reachedPoint(double max) {
        // if the current tick value (when negative) is greater than 10 ticks from the setpoint, return true
        if(m_motor.getSelectedSensorPosition() <= (RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple) + 10 && max < 0.0) {
            return true;
        }
        // if the current tick value (when positive) is greater than 10 ticks from the setpoint, return true
        else if(m_motor.getSelectedSensorPosition() >= (RotatorConstants.kRotatorDegreeLimit / RotatorConstants.kRotatorTickMultiple) - 10 && max > 0.0) {
            return true;
        }
        //otherwise, return false
        return false;
    }
}
