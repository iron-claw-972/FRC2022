package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shuffleboard extends SubsystemBase {
  private static Shuffleboard instance;

  public final SendableChooser<String> m_driveModeChooser = new SendableChooser<>();

  public static Shuffleboard getInstance() {
    if (instance == null) {
      instance = new Shuffleboard();
    }
    return instance;
  }

  public Shuffleboard() {

  }

}
