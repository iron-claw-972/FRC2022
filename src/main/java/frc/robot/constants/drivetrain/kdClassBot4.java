package frc.robot.constants.drivetrain;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class kdClassBot4 extends kdbase {
    public final static int 
    kLeftMotor1Port  = 0, 
    kRightMotor1Port = 15,
    kLeftMotor2Port  = -1, 
    kRightMotor2Port = -1,
    kLeftMotor3Port  = -1, 
    kRightMotor3Port = -1;

    //auto might be off becuse they were calbrated for classbot3
    public static final double kTrackWidthMeters = Units.inchesToMeters(20); // Distance between center of left wheel and center of right wheel in meters

    public static final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kGearRatio = 62.0 / 8.0;
    public static final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;
    public static final double kEncoderMetersPerSecond = kWheelDiameterMeters * Math.PI / kGearRatio * 10.0;

    // Use the SysId program in WPILib Tools to estimate values
    public static final double ksVolts = 0.59765; // Ks
    public static final double kvVoltSecondsPerMeter = 2.6544; // Kv
    public static final double kaVoltSecondsSquaredPerMeter = 0.15897; // Ka
    public static final double kRamseteP = 2.7489; // Kp for Ramsete PID
    public static final double kvVoltSecondsPerRadian = 0.05;
    public static final double kaVoltSecondsSquaredPerRadian = 0.005;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);


    // Velocity PID gain values
    public static final double kVelocityP = 1; // Proportional
    public static final double kVelocityI = 0; // Integral
    public static final double kVelocityD = 0; // Derivative


    //keep here for later
    // Teleop max speeds
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(10); // Max velocity
    public static final double kMaxAngularSpeedRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(60); // Max angular velocity

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);

    public static final boolean kRightEncoderReversed = false;
    public static final boolean kLeftEncoderReversed = true;
}
