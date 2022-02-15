package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static Limelight instance;

  private NetworkTable m_table;
  private String m_tableName;
  private boolean isConnected = false;

  private double tv;
  private double tx;
  private double ty;
  private double ta;

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  public Limelight() {
    m_tableName = "limelight";
    m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
  }

  public Limelight(String tableName) {
    m_tableName = tableName;
    m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
  }

  public Limelight(NetworkTable table) {
    m_table = table;
  }

  public boolean isConnected() {
    return isConnected;
  }

  public boolean hasValidTarget() {
    double tv = m_table.getEntry("tv").getDouble(0);
    return (tv == 1.0);
  }

  public double getHorizontalAngularError() {
    double tx = m_table.getEntry("tx").getDouble(0);
    return tx;
  }

  public double getVerticalAngularError() {
    double ty = m_table.getEntry("ty").getDouble(0);
    return ty;
  }

  public double getTargetArea() {
    double ta = m_table.getEntry("ta").getDouble(0);
    return ta;
  }

  public double getPipeline() {
    return 0.0;
  }

}
