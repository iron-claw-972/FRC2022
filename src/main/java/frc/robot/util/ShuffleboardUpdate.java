package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Driver;

public class ShuffleboardUpdate {
  

  
  public void update() {

    //drive mode
    SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());
    

  }
}
  