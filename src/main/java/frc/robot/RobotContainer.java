/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.kJoy;
import frc.robot.commands.ArcadeDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivetrain m_drive = new Drivetrain();

  static Joystick driver = new Joystick(kJoy.kDriverJoy);
  static Joystick operator = new Joystick(kJoy.kOperatorJoy);

  private static final JoystickButton driver_A = new JoystickButton(driver, 1),
    driver_B = new JoystickButton(driver, 2), driver_X = new JoystickButton(driver, 3),
    driver_Y = new JoystickButton(driver, 4), driver_LB = new JoystickButton(driver, 5),
    driver_RB = new JoystickButton(driver, 6), driver_BACK = new JoystickButton(driver, 7),
    driver_START = new JoystickButton(driver, 8);

  private static final JoystickButton operator_A = new JoystickButton(operator, 1),
    operator_B = new JoystickButton(operator, 2), operator_X = new JoystickButton(operator, 3),
    operator_Y = new JoystickButton(operator, 4), operator_LB = new JoystickButton(operator, 5),
    operator_RB = new JoystickButton(operator, 6), operator_BACK = new JoystickButton(operator, 7),
    operator_START = new JoystickButton(operator, 8);

  private static final POVButton driver_DPAD_UP = new POVButton(driver, 0),
    driver_DPAD_RIGHT = new POVButton(driver, 90), driver_DPAD_DOWN = new POVButton(driver, 180),
    driver_DPAD_LEFT = new POVButton(driver, 270);

  private static final POVButton operator_DPAD_UP = new POVButton(operator, 0),
    operator_DPAD_RIGHT = new POVButton(driver, 90), operator_DPAD_DOWN = new POVButton(operator, 180),
    operator_DPAD_LEFT = new POVButton(driver, 270);

  
  public RobotContainer() {

    m_drive.setDefaultCommand(new ArcadeDrive(m_drive));

    CameraServer.startAutomaticCapture();

    m_drive.setDefaultCommand(
      new RunCommand(() -> m_drive.arcadeDrive(getThrottleValue(), getTurnValue())));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons are created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick}),
   * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An spin in circle
    return new SequentialCommandGroup(
      new RunCommand(() -> m_drive.tankDrive(0.2, -0.2))
      );
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
    return -deadbandX(driver.getRawAxis(1), kJoy.kJoystickDeadband);
  }

  /**
   * Returns the deadbanded turn input from the main driver controller
   * 
   * @return the deadbanded turn input from the main driver controller
   */
  public static double getTurnValue() {
      // 5 represents left/right axis on the right joystick
      return deadbandX(driver.getRawAxis(5), kJoy.kJoystickDeadband);
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
}
