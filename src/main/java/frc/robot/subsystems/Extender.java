package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControllerFactory;
import frc.robot.Constants.ExtenderConstants;

public class Extender {
    private final WPI_TalonFX m_lmotor = ControllerFactory.createTalonFX(ExtenderConstants.kLeftExtenderPort);
    private final WPI_TalonFX m_rmotor = ControllerFactory.createTalonFX(ExtenderConstants.kRightExtenderPort);
    private int inverter = 1;

    public void inverter() {
        inverter *= -1;
        //if inverter is -1, smartdashboard is true and the pop-up box is green
        if(inverter == 1) {
            SmartDashboard.putBoolean("extender inverted", false);
        }
        if(inverter == -1) {
            SmartDashboard.putBoolean("extender inverted", true);
        }
    }

    public void runLeft(double pow) {
        //if ticks greater than a specified point, stop running
        
        m_lmotor.set(pow * inverter);
    }

    public void runRight(double pow) {
        m_rmotor.set(pow * inverter);
    }
}
