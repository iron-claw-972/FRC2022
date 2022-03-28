package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class BeltConstants {
  public final int kCargoBeltMotorPort = 6;

  public final double kOuttakeSpeed = 0.4;
  public final double kIntakeSpeed = -0.2;

  public final double kMotorClamp = 1;

  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 40;
  public final double kSupplyTriggerDuration = 0;
  public final NeutralMode kNeutral = NeutralMode.Brake;
}