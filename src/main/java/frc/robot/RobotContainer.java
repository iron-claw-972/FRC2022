/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.JoyConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivetrain m_drive = new Drivetrain();

  static Joystick m_driverController = new Joystick(JoyConstants.kDriverJoy);
  static Joystick m_operatorController = new Joystick(JoyConstants.kOperatorJoy);

  private static final JoystickButton m_driverController_A = new JoystickButton(m_driverController, 1),
      m_driverController_B = new JoystickButton(m_driverController, 2),
      m_driverController_X = new JoystickButton(m_driverController, 3),
      m_driverController_Y = new JoystickButton(m_driverController, 4),
      m_driverController_LB = new JoystickButton(m_driverController, 5),
      m_driverController_RB = new JoystickButton(m_driverController, 6),
      m_driverController_BACK = new JoystickButton(m_driverController, 7),
      m_driverController_START = new JoystickButton(m_driverController, 8);

  private static final JoystickButton m_operatorController_A = new JoystickButton(m_operatorController, 1),
      m_operatorController_B = new JoystickButton(m_operatorController, 2),
      m_operatorController_X = new JoystickButton(m_operatorController, 3),
      m_operatorController_Y = new JoystickButton(m_operatorController, 4),
      m_operatorController_LB = new JoystickButton(m_operatorController, 5),
      m_operatorController_RB = new JoystickButton(m_operatorController, 6),
      m_operatorController_BACK = new JoystickButton(m_operatorController, 7),
      m_operatorController_START = new JoystickButton(m_operatorController, 8);

  private static final POVButton m_driverController_DPAD_UP = new POVButton(m_driverController, 0),
      m_driverController_DPAD_RIGHT = new POVButton(m_driverController, 90),
      m_driverController_DPAD_DOWN = new POVButton(m_driverController, 180),
      m_driverController_DPAD_LEFT = new POVButton(m_driverController, 270);

  private static final POVButton m_operatorController_DPAD_UP = new POVButton(m_operatorController, 0),
      m_operatorController_DPAD_RIGHT = new POVButton(m_driverController, 90),
      m_operatorController_DPAD_DOWN = new POVButton(m_operatorController, 180),
      m_operatorController_DPAD_LEFT = new POVButton(m_driverController, 270);

  private Trajectory autonomousTrajectory;
  private boolean loadedAutonomousTrajectory = false;

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Arcade drive if no command given
    m_drive.setDefaultCommand(new ArcadeDrive(m_drive));

    // Start camera stream for driver
    CameraServer.startAutomaticCapture();

    // Attempt to load trajectory from PathWeaver
    loadAutonomousTrajectory(AutoConstants.kTrajectoryName);
  }

  /**
   * a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick}),
   * and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Drive at half speed when the right bumper is held
    m_driverController_RB
        .whenPressed(() -> m_drive.setMaxOutput(0.5))
        .whenReleased(() -> m_drive.setMaxOutput(1));

    m_driverController_B.whenPressed(() -> m_drive.modSensitivity()); // most likely remove this as does the same as above

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        m_drive.getFeedforward(),
        m_drive.getDriveKinematics(),
        Constants.kMaxVoltage);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_drive.getDriveKinematics())
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    if (loadedAutonomousTrajectory) {
      // Fallback to default trajectory
      autonomousTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at (1, 2) facing the +X direction
          new Pose2d(1, 2, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(4, 2, new Rotation2d(0)),
          // Pass config
          config);
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        autonomousTrajectory,
        m_drive::getPose,
        m_drive.getRamseteController(),
        m_drive.getFeedforward(),
        m_drive.getDriveKinematics(),
        m_drive::getWheelSpeeds,
        m_drive.getLeftRamsetePIDController(),
        m_drive.getRightRamsetePIDController(),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(autonomousTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  /**
   * Returns the deadbanded throttle input from the main driver controller
   * 
   * @return the deadbanded throttle input from the main driver controller
   */
  public static double getThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive. This method
    // corrects that by returning the opposite of the y-value
    // 1 represents up/down axis on the left joystick
    return -deadbandX(m_driverController.getRawAxis(1), JoyConstants.kJoystickDeadband);
  }

  /**
   * Returns the deadbanded turn input from the main driver controller
   * 
   * @return the deadbanded turn input from the main driver controller
   */
  public static double getTurnValue() {
    // 4 represents left/right axis on the right joystick
    return deadbandX(m_driverController.getRawAxis(4), JoyConstants.kJoystickDeadband);
  }

  /**
   * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
   * linear from (deadband, 0) to (1,1)
   * 
   * @param input    The input value to rescale
   * @param deadband The deadband
   * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
   */
  public static double deadbandX(double input, double deadband) {
    if (Math.abs(input) <= deadband) {
      return 0;
    } else if (Math.abs(input) == 1) {
      return input;
    } else {
      return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
    }
  }

  public void loadAutonomousTrajectory(String trajectoryName) {
    String trajectoryJSON = "paths/output/" + trajectoryName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      autonomousTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      loadedAutonomousTrajectory = true;
    } catch (IOException ex) {
      DriverStation.reportWarning(
          "Unable to open trajectory: " + trajectoryJSON + "\n" +
              "Falling back to default trajectory",
          ex.getStackTrace());
    }
  }
}
