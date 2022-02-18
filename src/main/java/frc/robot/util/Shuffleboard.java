package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.commands.*;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class Shuffleboard {
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  // Command pathweaver = Pathweaver.pathweaverCommand();

  public void setup() {
    

    
    autoChooser.setDefaultOption("pathweaver", new PathweaverCommand(Drivetrain.getInstance()));
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    // m_chooser.setDefaultOption("pathweaver", new DifferentialDrive(Drivetrain.getInstance()));
    autoChooser.addOption("Spin baby spin", new RunCommand(() -> Drivetrain.getInstance().tankDrive(0.5, -0.5), Drivetrain.getInstance()));

    

    SmartDashboard.putData(autoChooser);
  }

  public void update() {

    //drive mode
    SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());
    // System.out.println(m_chooser.getSelected().toString());

  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
  