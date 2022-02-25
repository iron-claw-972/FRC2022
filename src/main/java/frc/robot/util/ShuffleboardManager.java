package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.CargoShooter;
import frc.robot.subsystems.ClimbExtender;
import frc.robot.subsystems.ClimbRotator;

public class ShuffleboardManager {

  SendableChooser<Command> autoCommand = new SendableChooser<>();

  public void setup() {
    update();

  }

  public void update() {
    // driveMode();
    // subsystemSpam();
    // time();

  }

  public void time() {
    SmartDashboard.putNumber("Time Left", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Time Left Until Endgame", DriverStation.getMatchTime() - 30);
    SmartDashboard.putNumber("Auto Wait", 0);
  }

  public void driveMode() {
    autoCommand.setDefaultOption("pathweaver", Pathweaver.pathweaverCommand(AutoConstants.kTrajectoryName));
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    autoCommand.addOption("Spin baby spin", new RunCommand(() -> RobotContainer.m_drive.tankDrive(0.5, -0.5), RobotContainer.m_drive));
    autoCommand.addOption("fetch me my paper boy", new SequentialCommandGroup(new DriveDistance(9000, RobotContainer.m_drive), new DriveDistance(-9000, RobotContainer.m_drive)));
    // adds auto to shuffle board
    SmartDashboard.putData(autoCommand);
     
    // SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());
    SmartDashboard.putBoolean("Teleop", DriverStation.isTeleop());
  }

  public void subsystemSpam() {
    // put subsystem shuffleboard things in here!

    loadClimbExtenderShuffleboard(RobotContainer.m_extenderL);
    loadClimbExtenderShuffleboard(RobotContainer.m_extenderR);

    loadClimbRotatorShuffleboard(RobotContainer.m_climbRotatorL);
    loadClimbRotatorShuffleboard(RobotContainer.m_climbRotatorR);

    loadCargoShooterShuffleboard();
    loadCargoRotatorShuffleboard();
    loadCargoBeltShuffleboard();
    

  }

  public Command getAutonomousCommand() {
    driveMode();
    return autoCommand.getSelected();
  }

  public Command getAutonomousWaitCommand() {
    return new WaitCommand(SmartDashboard.getNumber("Auto Wait", 0));
  }

  public void loadCargoRotatorShuffleboard() {
    SmartDashboard.putNumber("Cargo Arm Angle", RobotContainer.m_cargoRotator.currentAngle());
    SmartDashboard.putBoolean("Cargo Rotator", RobotContainer.m_cargoRotator.isEnabled());
    SmartDashboard.putNumber("Raw Angle", RobotContainer.m_cargoRotator.currentAngleRaw());
    SmartDashboard.putData(RobotContainer.m_cargoRotator.cargoRotatorPID);
    SmartDashboard.putNumber("cargo rotator setpoint", RobotContainer.m_cargoRotator.getSetpoint());
  }

  public void loadCargoShooterShuffleboard() {
    SmartDashboard.putBoolean("Cargo Shooter", RobotContainer.m_cargoShooter.isEnabled());
    SmartDashboard.putData("CargoShooterPID", RobotContainer.m_cargoShooter.cargoShooterPID);
    SmartDashboard.putNumber("vel", RobotContainer.m_cargoShooter.getVelocity());
  }

  public void loadClimbExtenderShuffleboard(ClimbExtender extender) {
    SmartDashboard.putData("Climb Extender PID", extender.extenderPID);
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber(extender.getDirection() + " Extension", extender.currentExtension());
    // a pop-up in shuffleboard that states if the extender is on/off
    SmartDashboard.putBoolean(extender.getDirection() + " Extender", extender.isEnabled());
  }

  public void loadClimbRotatorShuffleboard(ClimbRotator rotator) {
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    SmartDashboard.putNumber(rotator.getDirection() + " Angle", rotator.currentAngle());
    // a pop-up in shuffleboard that states if the rotator is on/off
    SmartDashboard.putBoolean(rotator.getDirection() + " Rotator", rotator.isEnabled());
    // PID values that can be modified in shuffleboard
    SmartDashboard.putData("Climb Rotator PID", rotator.armPID);
    // zero value that can be modified in shuffleboard
    SmartDashboard.putNumber("Zero ClimbR", 80);
  }

  public void loadCargoBeltShuffleboard(){
    SmartDashboard.putBoolean("Cargo Belt", RobotContainer.m_cargoBelt.isEnabled());
  }

  public void loadBallDetectionShuffleboard(){
    SmartDashboard.putBoolean("Has Red Ball", RobotContainer.m_ballDetection.hasRedBall());
    SmartDashboard.putBoolean("Has Blue Ball", RobotContainer.m_ballDetection.hasBlueBall());
    SmartDashboard.putBoolean("Has Ball", RobotContainer.m_ballDetection.containsBall());
  }
}
  