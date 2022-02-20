package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.commands.*;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class ShuffleboardManager {

  
  SendableChooser<Command> autoCommand = new SendableChooser<>();

  // Command pathweaver = Pathweaver.pathweaverCommand();

  public void setup() {
    
    // add auto commands here.
    autoCommand.setDefaultOption("pathweaver", Pathweaver.pathweaverCommand(AutoConstants.kTrajectoryName));
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    // m_chooser.setDefaultOption("pathweaver", new DifferentialDrive(Drivetrain.getInstance()));
    autoCommand.addOption("Spin baby spin", new RunCommand(() -> RobotContainer.m_drive.tankDrive(0.5, -0.5), RobotContainer.m_drive));
    // adds atuo to shuffle board
    SmartDashboard.putData(autoCommand);
    SmartDashboard.putNumber("Auto Wait", 0);

    update();
  }

  public void update() {

    //drive mode
    SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());
    // System.out.println(m_chooser.getSelected().toString());
    SmartDashboard.putBoolean("Teleop", DriverStation.isTeleop());
    SmartDashboard.putNumber("Time Left", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Time Left Until Endgame", DriverStation.getMatchTime() - 30);

  }
  
  public Command getAutonomousCommand() {
    autoCommand.setDefaultOption("pathweaver", Pathweaver.pathweaverCommand(AutoConstants.kTrajectoryName));
    return autoCommand.getSelected();
  }

  public Command getAutonomousWaitCommand() {
    return new WaitCommand(SmartDashboard.getNumber("Auto Wait", 0));
  }

}
  