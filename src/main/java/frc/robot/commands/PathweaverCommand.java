package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathweaverCommand extends CommandBase {
    private final Drivetrain m_drive;
  
    Command command;

    public PathweaverCommand(Drivetrain subsystem) {
      m_drive = subsystem;
      addRequirements(subsystem);
      command = Pathweaver.pathweaverCommand();
    }
    
    @Override
    public void initialize() {
      command.initialize();
    }
    
    @Override
    public void execute() {
  
      command.execute();
    }

    @Override
    public void end(boolean interrupted) {
      command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
  
      return command.isFinished();
    }
  }