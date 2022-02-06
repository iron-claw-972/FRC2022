package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathweaverCommand extends CommandBase {
    private final Drivetrain m_drive;
  
    public PathweaverCommand(Drivetrain subsystem) {
      m_drive = subsystem;
      addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
      Pathweaver.pathweaverCommand().initialize();
    }
    
    @Override
    public void execute() {
  
      Pathweaver.pathweaverCommand().execute();
    }

    @Override
    public void end(boolean interrupted) {
      Pathweaver.pathweaverCommand().end(interrupted);
    }

    @Override
    public boolean isFinished() {
  
      return Pathweaver.pathweaverCommand().isFinished();
    }
  }