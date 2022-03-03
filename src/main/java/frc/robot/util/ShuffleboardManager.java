package frc.robot.util;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.subsystems.ClimbExtender;
import frc.robot.subsystems.ClimbRotator;
import frc.robot.commands.FlexibleAuto;
import frc.robot.controls.Driver;

public class ShuffleboardManager {

  SendableChooser<Command> autoCommand = new SendableChooser<>();
  ShuffleboardTab primaryTab = Shuffleboard.getTab("main");
  ShuffleboardTab pidTab = Shuffleboard.getTab("PID config");
  ShuffleboardTab autoTab = Shuffleboard.getTab("Auto config");
  
  NetworkTableEntry autoWait = autoTab.add("Auto Wait", 0.0).getEntry();
  NetworkTableEntry isFar = autoTab.add("Flexible Auto First Shoot is Fender", true).getEntry();
  NetworkTableEntry distance = autoTab.add("Flexible Auto Drive Distance", 0.0).getEntry();
  NetworkTableEntry intakeSecond = autoTab.add("Flexible Auto Get and Shoot a Second Time", true).getEntry();
  NetworkTableEntry limelightColor = primaryTab.add("Limelight (Red)", true).getEntry();
  

  public void setup() {
    primaryTab.addBoolean("Teleop", DriverStation::isTeleop);
    chooserUpdate();
    subsystemSpam();
    time();
    update();

    autoTab.add("Auto Chooser",autoCommand);
    primaryTab.addString("Drive Mode", this::getDriveModeString);
  }

  public void update() {
    
  }

  public void time() {
    primaryTab.addNumber("Time Left", DriverStation::getMatchTime);
    primaryTab.addNumber("Time Left Until Endgame" , this::getDriverStationTimeTillEndGame);
    // primaryTab.add("Auto Wait", 0);
  }
  private double getDriverStationTimeTillEndGame(){
    return DriverStation.getMatchTime() -30;
  }

  public void chooserUpdate() {
    autoCommand.setDefaultOption("pathweaver", Pathweaver.pathweaverCommand(AutoConstants.kTrajectoryName));
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    autoCommand.addOption("Spin baby spin", new RunCommand(() -> RobotContainer.m_drive.tankDrive(0.5, -0.5), RobotContainer.m_drive));
    autoCommand.addOption("fetch me my paper boy", new FlexibleAuto(isFar.getBoolean(true), distance.getDouble(0), intakeSecond.getBoolean(true)));
    // adds auto to shuffle board
    // SmartDashboard.putData("Auto Chooser",autoCommand);
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

    loadBallDetectionShuffleboard();
    

  }

  public Command getAutonomousCommand() {
    chooserUpdate();
    return autoCommand.getSelected();
  }
  public Command getAutonomousWaitCommand() {
    System.out.println(autoWait.getDouble(0));
    return new WaitCommand(autoWait.getDouble(0));
  }

  public void loadCargoRotatorShuffleboard() {
    primaryTab.addNumber("Cargo Arm Angle", RobotContainer.m_cargoRotator::currentAngle);
    primaryTab.addBoolean("Cargo Rotator", RobotContainer.m_cargoRotator::isEnabled);
    // primaryTab.addNumber("Cargo Arm Raw Angle", RobotContainer.m_cargoRotator::currentAngleRaw);
    primaryTab.addNumber("Cargo Rotator Setpoint", RobotContainer.m_cargoRotator::getSetpoint);

    pidTab.add("Cargo Rotator PID",RobotContainer.m_cargoRotator.cargoRotatorPID);
  }
  public void loadCargoShooterShuffleboard() {
    primaryTab.addBoolean("Cargo Shooter", RobotContainer.m_cargoShooter::isEnabled);
    // primaryTab.addNumber("Cargo Shooter Velocity", RobotContainer.m_cargoShooter::getVelocity);
    
    pidTab.add("CargoShooterPID", RobotContainer.m_cargoShooter.cargoShooterPID);
  }
  public void loadCargoBeltShuffleboard(){
    primaryTab.addBoolean("Cargo Belt", RobotContainer.m_cargoBelt::isEnabled);
  }

  public void loadClimbExtenderShuffleboard(ClimbExtender extender) {
    primaryTab.addNumber(extender.getSide() + " Extension", extender::currentExtension);
    primaryTab.addBoolean(extender.getSide() + " Extender", extender::isEnabled);
    
    pidTab.add(extender.getSide() + "Climb Extender PID", extender.extenderPID);
  }
  public void loadClimbRotatorShuffleboard(ClimbRotator rotator) {
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    primaryTab.addNumber(rotator.getSide() + " Climb Rotator Angle", rotator::currentAngle);
    // a pop-up in shuffleboard that states if the rotator is on/off
    primaryTab.addBoolean(rotator.getSide() + " Climb Rotator", rotator::isEnabled);

    primaryTab.addNumber(rotator.getSide() + " Climb Rotator Goal", rotator::getSetPoint);
    
    // PID values that can be modified in shuffleboard
    pidTab.add(rotator.getSide() + " Climb Rotator PID", rotator.armPID);
  }

  public void loadBallDetectionShuffleboard(){
    primaryTab.addBoolean("Has Red Ball", RobotContainer.m_ballDetection::hasRedBall);
    primaryTab.addBoolean("Has Blue Ball", RobotContainer.m_ballDetection::hasBlueBall);
    primaryTab.addBoolean("Has Ball", RobotContainer.m_ballDetection::containsBall);
    primaryTab.addBoolean("Has Ball Securely", RobotContainer.m_ballDetection::containsBallSecurely);

  }

  private String getDriveModeString(){
    return Driver.getDriveMode().toString();
  }

  public boolean getLimeightRed(){
    return limelightColor.getBoolean(true);
  }
  public boolean getLimeightBlue(){
    return !limelightColor.getBoolean(true);
  }
}
  