/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
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
  public static Controller m_controller = new Controller();

  
  public RobotContainer() {

    m_drive.setDefaultCommand(new RunDrive(m_drive));

    CameraServer.startAutomaticCapture();

    //m_drive.setDefaultCommand(
    //  new RunCommand(() -> m_drive.arcadeDrive(getThrottleValue(), getTurnValue())));
    
    // Configure the button bindings
    m_controller.configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons are created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick}),
   * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */


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
    return -driver.getRawAxis(1);
  }

  /**
   * Returns the deadbanded turn input from the main driver controller
   * 
   * @return the deadbanded turn input from the main driver controller
   */
  public static double getTurnValue() {
      // 4 represents left/right axis on the right joystick
      return driver.getRawAxis(4);
      // return -driver.getRawAxis(0);
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
