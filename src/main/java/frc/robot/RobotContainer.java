/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.controls.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.JoyConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DifferentialDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.*;
import frc.robot.autonomous.drivetrain.Pathweaver;

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

  // arm objects
  public Extender m_extenderLeft = new Extender(ExtenderConstants.kRightExtenderPort, true);
  public Extender m_extenderRight = new Extender(ExtenderConstants.kLeftExtenderPort, false);
  //-----//
  //public static Intake m_intake = new Intake();

  static Joystick m_driverController = new Joystick(JoyConstants.kDriverJoy);
  static Joystick m_operatorController = new Joystick(JoyConstants.kOperatorJoy);

  private static final JoystickButton m_driverController_A = new JoystickButton(m_driverController, 1),
      m_driverController_B = new JoystickButton(m_driverController, 2),
      m_driverController_X = new JoystickButton(m_driverController, 3),
      m_driverController_Y = new JoystickButton(m_driverController, 4),
      m_driverController_LB = new JoystickButton(m_driverController, 5),
      m_driverController_RB = new JoystickButton(m_driverController, 6),
      m_driverController_BACK = new JoystickButton(m_driverController, 7),
      m_driverController_START = new JoystickButton(m_driverController, 8),
      m_driverController_LJOYPRESS = new JoystickButton(m_driverController, 9),
      m_driverController_RJOYPRESS = new JoystickButton(m_driverController, 10);

  private static final JoystickButton m_operatorController_A = new JoystickButton(m_operatorController, 1),
      m_operatorController_B = new JoystickButton(m_operatorController, 2),
      m_operatorController_X = new JoystickButton(m_operatorController, 3),
      m_operatorController_Y = new JoystickButton(m_operatorController, 4),
      m_operatorController_LB = new JoystickButton(m_operatorController, 5),
      m_operatorController_RB = new JoystickButton(m_operatorController, 6),
      m_operatorController_BACK = new JoystickButton(m_operatorController, 7),
      m_operatorController_START = new JoystickButton(m_operatorController, 8),
      m_operatorController_LJOYPRESS = new JoystickButton(m_operatorController, 9),
      m_operatorController_RJOYPRESS = new JoystickButton(m_operatorController, 10);

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
  public Drivetrain m_drive = Drivetrain.getInstance();
  //public Intake m_intake = Intake.getInstance();

  public RobotContainer() {

    // default command to run in teleop
    m_drive.setDefaultCommand(new DifferentialDrive(m_drive));

    // Start camera stream for driver
    CameraServer.startAutomaticCapture();
    
    // Configure the button bindings
    Driver.configureButtonBindings();
    Operator.configureButtonBindings();
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
    // Extender motors spin upwards
    m_operatorController_DPAD_UP
      .whenPressed(() -> m_extenderLeft.set(ExtenderConstants.kExtenderMaxArmLength))
      .whenPressed(() -> m_extenderRight.set(ExtenderConstants.kExtenderMaxArmLength));
    // Extender motors spin downwards
    m_operatorController_DPAD_DOWN
      .whenPressed(() -> m_extenderLeft.set(0))
      .whenPressed(() -> m_extenderRight.set(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Attempt to load trajectory from PathWeaver
    Pathweaver.setupAutonomousTrajectory(AutoConstants.kTrajectoryName);
    return Pathweaver.pathweaverCommand();
  }
}
