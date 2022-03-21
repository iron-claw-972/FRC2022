package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

public class OldLog {

  DataLog log;

  DoubleLogEntry extenderLExtension;
  BooleanLogEntry extenderLEnabled;

  public void start(){
    DataLogManager.start();
    DataLog log = DataLogManager.getLog();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void initiate() {
    extenderLExtension = new DoubleLogEntry(log, "/leftExtender/extension");
    extenderLEnabled = new BooleanLogEntry(log, "/leftExtender/enabled");
  }

  public void update() {
    extenderLExtension.append(RobotContainer.m_extenderL.currentExtensionRaw());
    extenderLEnabled.append(RobotContainer.m_extenderL.isEnabled());
  }






  private Supplier supplier;

  public OldLog(Supplier value, String name) {
    supplier = value;
    
  }
  
}