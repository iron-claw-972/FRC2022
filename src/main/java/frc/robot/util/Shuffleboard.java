package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.commands.*;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class Shuffleboard {
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  // Command pathweaver = Pathweaver.pathweaverCommand();

  public void setup() {
    

    
    m_chooser.setDefaultOption("pathweaver", new PathweaverCommand(Drivetrain.getInstance()));
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    // m_chooser.setDefaultOption("pathweaver", new DifferentialDrive(Drivetrain.getInstance()));
    m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));

    SmartDashboard.putData(m_chooser);
  }

  public void update() {

    //drive mode
    SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());
    System.out.println(m_chooser.getSelected().toString());

  }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
  