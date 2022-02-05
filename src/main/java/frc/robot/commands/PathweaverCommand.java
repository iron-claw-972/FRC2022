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
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
  
      Pathweaver.pathweaverCommand().execute();
    }

    @Override
    public boolean isFinished() {
  
      return Pathweaver.pathweaverCommand().isFinished();
    }
  }