package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.commands.*;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class Shuffleboard {
  
  // Command pathweaver = Pathweaver.pathweaverCommand();
  public void setup() {
    

    SendableChooser<Command> m_chooser = new SendableChooser<>();
    // m_chooser.setDefaultOption("pathweaver", Pathweaver.pathweaverCommand());
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    m_chooser.setDefaultOption("pathweaver", new DifferentialDrive(Drivetrain.getInstance()));
    m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));

    SmartDashboard.putData(m_chooser);
  }

  public void update() {

    //drive mode
    SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());

  }
}
  