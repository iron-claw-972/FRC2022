package frc.robot.util;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.controls.Driver;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.ClimbExtender;
import frc.robot.subsystems.ClimbRotator;

public class ShuffleboardManager {

  SendableChooser<Command> autoCommand = new SendableChooser<>();
  ShuffleboardTab primaryTab = Shuffleboard.getTab("main");
  ShuffleboardTab pidTab = Shuffleboard.getTab("PID config");
  
  NetworkTableEntry autoWait = primaryTab.add("Auto Wait", 0).getEntry();

  public void setup() {
    primaryTab.addBoolean("Teleop", DriverStation::isTeleop);
    driveMode();
    subsystemSpam();
    time();
    update();

    primaryTab.add("Auto Chooser",autoCommand);
  }

  public void update() {
    // driveMode();
    // subsystemSpam();
    // time();
  }

  public void time() {
    primaryTab.addNumber("Time Left", DriverStation::getMatchTime);
    primaryTab.addNumber("Time Left Until Endgame" , this::getDriverStationTimeTillEndGame);
    // primaryTab.add("Auto Wait", 0);
  }

  private double getDriverStationTimeTillEndGame(){
    return DriverStation.getMatchTime() -30;
  }

  public void driveMode() {
    autoCommand.setDefaultOption("pathweaver", Pathweaver.pathweaverCommand(AutoConstants.kTrajectoryName));
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    autoCommand.addOption("Spin baby spin", new RunCommand(() -> RobotContainer.m_drive.tankDrive(0.5, -0.5), RobotContainer.m_drive));
    autoCommand.addOption("fetch me my paper boy", new SequentialCommandGroup(new DriveDistance(9000, RobotContainer.m_drive), new DriveDistance(-9000, RobotContainer.m_drive)));
    // adds auto to shuffle board
    // SmartDashboard.putData("Auto Chooser",autoCommand);
     
    SmartDashboard.putString("Drive Mode", Driver.getDriveMode().toString());
    SmartDashboard.putBoolean("Teleop", DriverStation.isTeleop());
  }

  public void subsystemSpam() {
    // put subsystem shuffleboard things in here!

    // loadClimbExtenderShuffleboard(RobotContainer.m_extenderL);
    // loadClimbExtenderShuffleboard(RobotContainer.m_extenderR);

    // loadClimbRotatorShuffleboard(RobotContainer.m_climbRotatorL);
    // loadClimbRotatorShuffleboard(RobotContainer.m_climbRotatorR);

    loadCargoShooterShuffleboard();
    loadCargoRotatorShuffleboard();
    loadCargoBeltShuffleboard();
    

  }

  public Command getAutonomousCommand() {
    driveMode();
    return autoCommand.getSelected();
  }

  public Command getAutonomousWaitCommand() {
    System.out.println(autoWait.getDouble(0));
    return new WaitCommand(autoWait.getDouble(0));
  }

  public void loadCargoRotatorShuffleboard() {
    primaryTab.addNumber("Cargo Arm Angle", RobotContainer.m_cargoRotator::currentAngle);
    primaryTab.addBoolean("Cargo Rotator", RobotContainer.m_cargoRotator::isEnabled);
    primaryTab.addNumber("Raw Angle", RobotContainer.m_cargoRotator::currentAngleRaw);
    primaryTab.addNumber("cargo rotator setpoint", RobotContainer.m_cargoRotator::getSetpoint);

    pidTab.add("Cargo Rotator PID",RobotContainer.m_cargoRotator.cargoRotatorPID);
  }

  public void loadCargoShooterShuffleboard() {
    primaryTab.addBoolean("Cargo Shooter", RobotContainer.m_cargoShooter::isEnabled);
    primaryTab.addNumber("vel", RobotContainer.m_cargoShooter::getVelocity);
    primaryTab.addNumber("cargo rotator setpoint", RobotContainer.m_cargoRotator::getSetpoint);
    
    pidTab.add("CargoShooterPID", RobotContainer.m_cargoShooter.cargoShooterPID);
  }

  public void loadClimbExtenderShuffleboard(ClimbExtender extender) {
    primaryTab.addNumber(extender.getDirection() + " Extension", extender::currentExtension);
    primaryTab.addBoolean(extender.getDirection() + " Extender", extender::isEnabled);
    
    pidTab.add(extender.getDirection() + "Climb Extender PID", extender.extenderPID);
  }

  public void loadClimbRotatorShuffleboard(ClimbRotator rotator) {
    primaryTab.addNumber(rotator.getDirection() + " Angle", rotator::currentAngle);
    primaryTab.addBoolean(rotator.getDirection() + " Rotator", rotator::isEnabled);
    
    // PID values that can be modified in shuffleboard
    pidTab.add(rotator.getDirection() + " Climb Rotator PID", rotator.armPID);
  }

  public void loadCargoBeltShuffleboard(){
    primaryTab.addBoolean("Cargo Belt", RobotContainer.m_cargoBelt::isEnabled);
  }

  public void loadBallDetectionShuffleboard(){
    primaryTab.addBoolean("Has Red Ball", RobotContainer.m_ballDetection::hasRedBall);
    primaryTab.addBoolean("Has Blue Ball", RobotContainer.m_ballDetection::hasBlueBall);
    primaryTab.addBoolean("Has Ball", RobotContainer.m_ballDetection::containsBall);
  }

}
  