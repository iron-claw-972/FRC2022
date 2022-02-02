package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ExtenderConstants;


public class Extender extends SubsystemBase{
    final WPI_TalonFX m_motor;
    double setpoint = ExtenderConstants.kExtenderSetpoint;
    
    private final PIDController extenderPID = new PIDController(1, 0, 0);

    public Extender(int port, boolean left) {
        m_motor = ControllerFactory.createTalonFX(port);

        if(left == true) {
            // so that encoder values aren't negative
            m_motor.setSensorPhase(true);
            // so that the arm doesn't spin in an opposing direction
            m_motor.setInverted(true);
        }

        // the lowest tick limit is 0, and must be checked every 10 milliseconds
        m_motor.configReverseSoftLimitThreshold(0, 10);

        // converts the length of the arm in inches to ticks and makes that the maximum tick limit, it's checked every 10 milliseconds
        m_motor.configForwardSoftLimitThreshold(ExtenderConstants.kExtenderMaxArmLength / ExtenderConstants.kExtenderTickMultiple, 10);

        // every time the robot is started, arm MUST start at maximum compression in order to maintain consistency
        // TODO: Make this better.
        m_motor.setSelectedSensorPosition(0.0);
    }

    public boolean reachedTopPoint() {
        // if the current tick value is less than the maximum (135 ticks) by 10, return true
        if(m_motor.getSelectedSensorPosition() >= (ExtenderConstants.kExtenderMaxArmLength / ExtenderConstants.kExtenderTickMultiple) - 10) {
            return true;
        }
        // otherwise, return false
        return false;
    }

    public boolean reachedBottomPoint() {
        // if the current tick value is greater than the minimum (0) by 10, return true
        if(m_motor.getSelectedSensorPosition() <= (10)) {
            return true;
        }
        // otherwise, return false
        return false;
    }

    public void set(double distance){
        setpoint = distance;
    }


    @Override
    public void periodic(){
        m_motor.set(extenderPID.calculate(m_motor.getSelectedSensorPosition() * ExtenderConstants.kExtenderTickMultiple, setpoint));

        // a pop-up in shuffleboard that allows you to see how much the arm extended in feet
        SmartDashboard.putNumber("Current Extension (Inches)", m_motor.getSelectedSensorPosition() * ExtenderConstants.kExtenderTickMultiple);

        //so we know the value
        System.out.println(setpoint);
    }

    
} 
