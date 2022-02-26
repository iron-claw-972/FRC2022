/*----------------------------------------------------------------------------
Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        
Open Source Software - may be modified and shared by FRC teams. The code   
must be accompanied by the FIRST BSD license file in the root directory of 
the project.                                                               
----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.util.ShooterMethods;
import frc.robot.util.ShuffleboardManager;
import frc.robot.commands.DifferentialDrive;
import frc.robot.commands.GetDistance;
import frc.robot.commands.TeleopDrive;
import frc.robot.controls.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.cameraserver.CameraServer;

/*
  This class is where the bulk of the robot should be declared. Since
  Command-based is a
  "declarative" paradigm, very little robot logic should actually be handled in
  the {@link Robot}
  periodic methods (other than the scheduler calls). Instead, the structure of
  the robot
  (including subsystems, commands, and button mappings) should be declared
  here.
 */

 public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static ShuffleboardManager m_shuffleboard = new ShuffleboardManager();
  public static BallDetection m_ballDetection = new BallDetection();

  public static Drivetrain m_drive = new Drivetrain();

  public static ClimbRotator m_climbRotatorR = new ClimbRotator(false);
  public static ClimbRotator m_climbRotatorL = new ClimbRotator(true);
  public static ClimbExtender m_extenderR = new ClimbExtender(false);
  public static ClimbExtender m_extenderL = new ClimbExtender(true);
  public static CargoRotator m_cargoRotator = new CargoRotator();
  public static CargoBelt m_cargoBelt = new CargoBelt();
  public static CargoShooter m_cargoShooter = new CargoShooter();
  public static BallDetection m_balldetector = new BallDetection();

  public static Limelight m_limelight = new Limelight(() -> ShooterMethods.isArmFront());


  public RobotContainer() {
    

    // default command to run in teleop
    
    m_drive.setDefaultCommand(new TeleopDrive(m_drive));
    // m_testArm.setDefaultCommand(new armPID(m_testArm));
    //m_cargoShooter.setDefaultCommand(new RunCommand(() -> RobotContainer.m_cargoShooter.setOutput(Operator.controller.getJoystickAxis().leftY()), m_cargoShooter));
    //m_cargoBelt.setDefaultCommand(new RunCommand(() -> RobotContainer.m_cargoBelt.setOutput(-Operator.controller.getJoystickAxis().rightY()), m_cargoBelt));
    // m_limelight.setDefaultCommand(new GetDistance(m_limelight, m_cargoRotator));


    // Start camera stream for driver
    CameraServer.startAutomaticCapture();
    
    // Configure the button bindings
    Driver.configureButtonBindings();
    Operator.configureButtonBindings();
    // TestingJoystick.configureButtonBindings();

    //sets up shuffle board
  }

  /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Attempt to load trajectory from PathWeaver
    return m_shuffleboard.getAutonomousCommand();
    // return new SequentialCommandGroup(
    //   m_shuffleboard.getAutonomousWaitCommand(),
    //   m_shuffleboard.getAutonomousCommand()
    // );
  }
}