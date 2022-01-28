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
import frc.robot.controls.*;
import frc.robot.controls.Functions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;

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
  public static Intake m_intake = new Intake();

  public static Functions m_controller = new Functions();

  public Functions controls;

  public RobotContainer() {
    // Configure the button bindings

    m_drive.setDefaultCommand(new RunDrive(m_drive));

    // Start camera stream for driver
    CameraServer.startAutomaticCapture();

    //m_drive.setDefaultCommand(
    //  new RunCommand(() -> m_drive.arcadeDrive(getThrottleValue(), getTurnValue())));
    
    // Configure the button bindings
    Driver.configureButtonBindings();
    // Attempt to load trajectory from PathWeaver
    
  }

  /**
   * a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick}),
   * and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Pathweaver.loadAutonomousTrajectory(AutoConstants.kTrajectoryName);
    
    // Run path following command, then stop at the end. At the same time intake.
    // "Deadline" is the first command, 
    // meaning the whole group will stop once the first command does.
    return Pathweaver.pathweaverCommand();
    //return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
