package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Driver;

public class ShuffleboardUpdate extends CommandBase {
    @Override
    public void execute() {

      //drive mode
      SmartDashboard.putBoolean("arcade drive", Driver.isDrive("arcade"));
      SmartDashboard.putBoolean("prop drive", Driver.isDrive("prop"));
      SmartDashboard.putBoolean("shift drive", Driver.isDrive("shift"));
      



    }
  }
  