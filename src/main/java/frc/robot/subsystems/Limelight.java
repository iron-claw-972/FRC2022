package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robotConstants.limelight.MarinusLimelightConstants;

public class Limelight extends SubsystemBase {
  public static MarinusLimelightConstants constants = new MarinusLimelightConstants();

  private NetworkTable m_table;
  private String m_tableName;

  private boolean m_hasValidTarget; // Whether target is detected
  private double m_horizontalAngularOffset; // Horizontal angular displacement of target from crosshair
  private double m_verticalAngularOffset; // Vertical angular displacement of target from crosshair
  private double m_targetArea; // Target area
  private double m_skew; // Skew or rotation
  private Pipeline m_pipeline;
  private double m_latency;
  private boolean m_isDriverCamera = false;
  private StreamMode m_streamMode = StreamMode.MAIN;
  private LEDMode m_ledMode = LEDMode.OFF;

  // private MedianFilter m_TxMedianFilter = new MedianFilter(5);
  // private MedianFilter m_TyMedianFilter = new MedianFilter(5);
  private Debouncer m_TvDebouncer = new Debouncer(0.05, DebounceType.kBoth);

  BooleanSupplier m_getIsFacingFront;

  public Limelight(BooleanSupplier getIsFacingFront) {
    m_tableName = "limelight";
    m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    m_getIsFacingFront = getIsFacingFront;
  }

  public Limelight(String tableName, BooleanSupplier getIsFacingFront) {
    m_tableName = tableName;
    m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    m_getIsFacingFront = getIsFacingFront;
  }

  public Limelight(NetworkTable table, BooleanSupplier getIsFacingFront) {
    m_table = table;
    m_getIsFacingFront = getIsFacingFront;
  }

  @Override
  public void periodic() {
    updateData();
  }

  private void updateData() {
    // setCameraMode(m_isDriverCamera);
    // setLedMode(m_ledMode);
    // setStreamMode(m_streamMode);
    // m_latency = getTl();

    if (!m_isDriverCamera) {
      m_pipeline = getPipeline();
      m_hasValidTarget = m_TvDebouncer.calculate(getTv() == 1.0);

      if (m_hasValidTarget) {
        // m_targetArea = getTa();
        // m_skew = getTs();

        if (constants.kIsMountedHorizontally) {
          m_horizontalAngularOffset = getTx();
          m_verticalAngularOffset = getTy();
        } else {
          m_horizontalAngularOffset = getTy();
          m_verticalAngularOffset = getTx();
        }

        if (!isFacingFront()) {
          // Negate values if limelight is upside down
          m_horizontalAngularOffset *= -1;
          m_verticalAngularOffset *= -1;
          m_skew *= -1;
        }
      } else {
        setNullValues();
      }
    } else {
      setNullValues();
    }
  }

  private double getTl() {
    return m_table.getEntry("tl").getDouble(Double.NaN);
  }

  private boolean isFacingFront() {
    return m_getIsFacingFront.getAsBoolean();
  }

  private void setNullValues() {
    m_targetArea = Double.NaN;
    m_skew = Double.NaN;
    m_horizontalAngularOffset = Double.NaN;
    m_verticalAngularOffset = Double.NaN;
  }

  public enum LEDMode {
    PIPELINE, OFF, BLINK, ON
  }

  public void setLedMode(LEDMode ledMode) {
    switch (ledMode) {
      case PIPELINE:
        m_table.getEntry("ledMode").setNumber(0);
        m_ledMode = ledMode;
        break;
      case OFF:
        m_table.getEntry("ledMode").setNumber(1);
        m_ledMode = ledMode;
        break;
      case BLINK:
        m_table.getEntry("ledMode").setNumber(2);
        m_ledMode = ledMode;
        break;
      case ON:
        m_table.getEntry("ledMode").setNumber(3);
        m_ledMode = ledMode;
        break;
      default:
        m_table.getEntry("ledMode").setNumber(0);
        m_ledMode = LEDMode.PIPELINE;
    }
  }

  public void setCameraMode(boolean isDriverCamera) {
    if (isDriverCamera) {
      m_table.getEntry("camMode").setNumber(1);
      m_isDriverCamera = true;
    } else {
      m_table.getEntry("camMode").setNumber(0);
      m_isDriverCamera = false;
    }
  }

  public boolean isDriverCamera() {
    return m_isDriverCamera;
  }

  private double getTv() {
    double tv = m_table.getEntry("tv").getDouble(Double.NaN);
    return tv;
  }

  private double getTs() {
    double ts = m_table.getEntry("ts").getDouble(Double.NaN);
    return ts;
  }

  private double getTx() {
    double tx = m_table.getEntry("tx").getDouble(Double.NaN);
    // return m_TxMedianFilter.calculate(tx);
    return tx;
  }

  private double getTy() {
    double ty = m_table.getEntry("ty").getDouble(Double.NaN);
    // return m_TyMedianFilter.calculate(ty);
    return ty;
  }

  private double getTa() {
    double ta = m_table.getEntry("ta").getDouble(Double.NaN);
    return ta;
  }

  public double getSkew() {
    return m_skew;
  }

  public double getLatency() {
    return m_latency;
  }

  public Pipeline getPipeline() {
    double pipeline = m_table.getEntry("getpipe").getDouble(Double.NaN);
    switch ((int) Math.round(pipeline)) {
      case 0:
        return Pipeline.RED_CARGO;
      case 1:
        return Pipeline.BLUE_CARGO;
      case 2:
        return Pipeline.UPPER_HUB;
      case 3:
        return Pipeline.DRIVER;
      default:
        return Pipeline.NO_PIPELINE;
    }
  }

  public enum Pipeline {
    RED_CARGO, BLUE_CARGO, UPPER_HUB, DRIVER, NO_PIPELINE
  }

  private void setPipeline(Pipeline pipeline) {
    m_isDriverCamera = false;
    m_ledMode = LEDMode.PIPELINE;

    switch (pipeline) {
      case RED_CARGO:
        m_table.getEntry("pipeline").setNumber(constants.kRedCargoPipeline);
        break;
      case BLUE_CARGO:
        m_table.getEntry("pipeline").setNumber(constants.kBlueCargoPipeline);
        break;
      case UPPER_HUB:
        m_table.getEntry("pipeline").setNumber(constants.kUpperHubPipeline);
        break;
      case DRIVER:
        m_table.getEntry("pipeline").setNumber(constants.kDriverPipeline);
        break;
      default:
    }
    updateData();
  }

  public StreamMode getStreamMode() {
    double streamMode = m_table.getEntry("stream").getDouble(Double.NaN);
    switch ((int) Math.round(streamMode)) {
      case 0:
        return StreamMode.STANDARD;
      case 1:
        return StreamMode.MAIN;
      case 2:
        return StreamMode.SECONDARY;
      default:
        return StreamMode.STANDARD;
    }
  }

  public enum StreamMode {
    STANDARD, MAIN, SECONDARY
  }

  private void setStreamMode(StreamMode streamMode) {
    switch (streamMode) {
      case STANDARD:
        m_table.getEntry("stream").setNumber(0);
        m_streamMode = streamMode;
        break;
      case MAIN:
        m_table.getEntry("stream").setNumber(1);
        m_streamMode = streamMode;
        break;
      case SECONDARY:
        m_table.getEntry("stream").setNumber(2);
        m_streamMode = streamMode;
        break;
      default:
    }
  }

  private double getLimelightHeight(double armAngle) {
    return constants.kPivotHeight
        + (constants.kPivotToLimelightLength * Math.sin(Units.degreesToRadians(armAngle)));
  }

  private double getLimelightFaceAngle(double armAngle) {
    // If angle is obtuse, find the supplementary angle
    double limelightFaceAngle = armAngle + constants.kStipeToLimelightFaceAngularOffset;
    if (limelightFaceAngle < 90) {
      return (limelightFaceAngle);
    }
    return 180 - (limelightFaceAngle);
  }

  public double getTargetArea() {
    return m_targetArea;
  }

  public boolean hasValidTarget() {
    return m_hasValidTarget;
  }

  public void setBallPipeline(boolean isRedBall) {
    Pipeline pipeline;
    if (isRedBall) {
      pipeline = Pipeline.RED_CARGO;
    } else {
      pipeline = Pipeline.BLUE_CARGO;
    }
    setPipeline(pipeline);
  }

  public void setUpperHubPipeline() {
    setPipeline(Pipeline.UPPER_HUB);
  }

  public void setDriverPipeline() {
    setPipeline(Pipeline.DRIVER);
  }

  public double getHubDistance(double stipeAngle) {
    return getDistance(stipeAngle, constants.kHubHeight);
  }

  public double getBallDistance(double stipeAngle, boolean isRedBall) {
    return getDistance(stipeAngle, constants.kBallTargetHeight);
  }

  private double getDistance(double stipeAngle, double targetHeight) {
    double limelightAngleRad = Units.degreesToRadians(getLimelightFaceAngle(stipeAngle) + m_verticalAngularOffset);
    double distance = ((targetHeight - getLimelightHeight(stipeAngle)) / (Math.tan(limelightAngleRad)));

    return distance;
  }

  public double getHubHorizontalAngularOffset() {
    return m_horizontalAngularOffset;
  }

  public double getBallHorizontalAngularOffset(boolean isRedBall) {
    return m_horizontalAngularOffset;
  }

  public double getVerticalAngularOffset() {
    return m_verticalAngularOffset;
  }
}
