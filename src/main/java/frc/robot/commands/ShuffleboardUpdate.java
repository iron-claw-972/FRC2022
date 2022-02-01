package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Driver;

public class ShuffleboardUpdate extends CommandBase {
    @Override
    public void execute() {

      //drive mode
      SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());

    }
  }
  