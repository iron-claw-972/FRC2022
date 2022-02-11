package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.Driver;

public class ShuffleboardUpdate extends CommandBase {
  @Override
  public void initialize(){
    SmartDashboard.putNumber("P", 0.1);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
    
  }
  
  @Override
  public void execute() {

    //drive mode
    SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());

    //SmartDashboard.putBoolean("Has Ball", RobotContainer.m_ballDetection.containsBall());
  }
}
  