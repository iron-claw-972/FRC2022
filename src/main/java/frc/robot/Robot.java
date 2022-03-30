/*----------------------------------------------------------------------------
 Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        
 Open Source Software - may be modified and shared by FRC teams. The code   
 must be accompanied by the FIRST BSD license file in the root directory of 
 the project.                                                               
----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.DifferentialDrive;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.controls.*;
import frc.robot.subsystems.*;
import frc.robot.util.Log;
import frc.robot.util.ShooterMethods;
import frc.robot.util.ShuffleboardManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoCommand;

  public static ShuffleboardManager shuffleboard = new ShuffleboardManager();
  public static Drivetrain drive = new Drivetrain();
  public static Rotator rotatorL = new Rotator(true);
  public static Rotator rotatorR = new Rotator(false);
  public static Extender extenderL = new Extender(true);
  public static Extender extenderR = new Extender(false);
  public static Arm arm = new Arm();
  public static Belt belt = new Belt();
  public static Shooter shooter = new Shooter();
  public static BallDetection ballDetection = new BallDetection();
  public static Log log = new Log();
  
  UsbCamera m_camera1;
  UsbCamera m_camera2;

  public static Limelight limelight = new Limelight(ShooterMethods::isLimelightFaceFront);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //setup cameras 
    m_camera1 = CameraServer.startAutomaticCapture();
    m_camera2 = CameraServer.startAutomaticCapture();

    int factor = 10; // max is 80
    int width = 16 * factor;
    int height = 9 * factor;
    
    // m_camera1.setFPS(30);
    // m_camera1.setResolution(width, height);
    // m_camera2.setFPS(30);
    // m_camera2.setResolution(width, height);

    // default command to run in teleop
    
    drive.setDefaultCommand(new DifferentialDrive(drive));
    // m_testArm.setDefaultCommand(new armPID(m_testArm));
    //m_cargoShooter.setDefaultCommand(new RunCommand(() -> m_cargoShooter.setOutput(Operator.controller.getJoystickAxis().leftY()), m_cargoShooter));
    //m_cargoBelt.setDefaultCommand(new RunCommand(() -> m_cargoBelt.setOutput(-Operator.controller.getJoystickAxis().rightY()), m_cargoBelt));
    // m_limelight.setDefaultCommand(new GetDistance(m_limelight, m_cargoRotator));
    
    // Configure the button bindings
    DriverStation.silenceJoystickConnectionWarning(true);

    Driver.configureControls();
    Operator.configureControls();
    // ClimbOperator.configureButtonBindings();

    //sets up shuffle board
    shuffleboard.setup();
    log.initialize();
    // m_autonomousCommand = m_getAutonomousCommand();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    shuffleboard.update();
    drive.updateMotors();
    log.updateBuffer();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    ShooterMethods.disableArm();
    ShooterMethods.disableShiitake();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link Robot} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_getAutonomousCommand();
    m_autoCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
      //commented out for safety so that no one dies
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
    drive.setHalfCoast();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shuffleboard.getAutonomousCommand();
  }
}
