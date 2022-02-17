package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robotConstants.limelight.TraversoLimelightConstants;

public class Limelight extends SubsystemBase {
  public static TraversoLimelightConstants constants = new TraversoLimelightConstants();

  private static Limelight instance;

  private NetworkTable m_table;
  private String m_tableName;
  private boolean isConnected = false;

  private double tv; // Whether target is detected
  private double tx; // Horizontal angular displacement of target from crosshair
  private double ty; // Vertical angular displacement of target from crosshair
  private double ta; // Target area
  private double pipeline;

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

  private boolean isConnected() {
    return isConnected;
  }

  private boolean hasValidTarget() {
    double tv = m_table.getEntry("tv").getDouble(0);
    return (tv == 1.0);
  }

  private double getHorizontalAngularOffset() {
    double tx = m_table.getEntry("tx").getDouble(0);
    return tx;
  }

  private double getVerticalAngularoffset() {
    double ty = m_table.getEntry("ty").getDouble(0);
    return ty;
  }

  private double getTargetArea() {
    double ta = m_table.getEntry("ta").getDouble(0);
    return ta;
  }

  private double getPipeline() {
    /*
      Pipeline 0: Red Cargo
      Pipeline 1: Blue Cargo
      Pipeline 2: Upper Hub
    */
    return m_table.getEntry("getpipe").getDouble(0);
  }

  private void setPipeline(double pipeline) {
    
  }

  private double getLimelightHeight() {
    return 0.0;
  }

  private double getLimelightAngle(double armAngle) {
    return 0.0;
  }

  public double getHubDistance(double armAngle) {
    return (constants.kTargetHeight - getLimelightHeight()) / (Math.tan(Units.degreesToRadians(getLimelightAngle(armAngle) + getVerticalAngularoffset())));
  }

  public double getHubOffset() {
    return 0.0;
  }

  public double getBallOffset(boolean isRedBall) {
    return 0.0;
  }

  public void setLight(boolean on) {

  }
}
