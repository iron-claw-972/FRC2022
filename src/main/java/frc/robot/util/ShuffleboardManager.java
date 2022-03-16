package frc.robot.util;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.subsystems.ClimbExtender;
import frc.robot.subsystems.ClimbRotator;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.ChaseBall;
import frc.robot.commands.FlexibleAuto;
import frc.robot.commands.GetDistance;
import frc.robot.controls.Driver;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;

public class ShuffleboardManager {

  SendableChooser<Command> autoCommand = new SendableChooser<>();
  public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  public static ShuffleboardTab cargoTab = Shuffleboard.getTab("Cargo");
  public static ShuffleboardTab climbTab = Shuffleboard.getTab("Climb");
  public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry autoWait = autoTab.add("Auto Wait", 0.0).getEntry();
  NetworkTableEntry isFar = autoTab.add("Flexible Auto First Shoot is Fender", true).getEntry();
  NetworkTableEntry distance = autoTab.add("Flexible Auto Drive Distance", 0.0).getEntry();
  NetworkTableEntry intakeSecond = autoTab.add("Flexible Auto Get a Second Time", true).getEntry();
  NetworkTableEntry shootSecond = autoTab.add("Flexible Auto Shoot a Second Time", true).getEntry();
  NetworkTableEntry limelightColor = cargoTab.add("Limelight (Red)", true).getEntry();
  
  TraversoClimbExtenderConstants extenderConstants = new TraversoClimbExtenderConstants();
  

  public void setup() {
    mainTab.addBoolean("Is Teleop", DriverStation::isTeleop);
    mainTab.addNumber("left drive encoder", RobotContainer.m_drive::getLeftPosition);
    climbTab.addNumber("Max Extension Ticks", () -> extenderConstants.kExtenderMaxArmTicks);
    chooserUpdate();
    subsystemSpam();
    time();
    update();

    autoTab.add("Auto Chooser", autoCommand);
    mainTab.addString("Drive Mode", this::getDriveModeString);
  }

  public void update() {
    
  }

  public void time() {
    mainTab.addNumber("Time Left", DriverStation::getMatchTime);
    mainTab.addNumber("Time Left Until Endgame" , this::getDriverStationTimeTillEndGame);
    // primaryTab.add("Auto Wait", 0);
  }
  private double getDriverStationTimeTillEndGame(){
    return DriverStation.getMatchTime() -30;
  }

  public void chooserUpdate() {
    // originally 0.8492
    autoCommand.setDefaultOption("fetch me my paper boy", new FlexibleAuto(0.6642, true, true, Constants.kIsRedAlliance));
    // autoCommand.setDefaultOption("fetch me my paper boy", new FlexibleAuto(distance.getDouble(0), intakeSecond.getBoolean(true), shootSecond.getBoolean(true), limelightColor.getBoolean(Constants.kIsRedAlliance)));
    autoCommand.addOption("pathweaver", Pathweaver.pathweaverCommand(AutoConstants.kTrajectoryName));
    // m_chooser.addOption("teleop", new TeleopDrive(Drivetrain.getInstance()));
    autoCommand.addOption("Spin baby spin", new RunCommand(() -> RobotContainer.m_drive.tankDrive(0.5, -0.5), RobotContainer.m_drive));
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
    loadLimelightShuffleboard();
    

  }

  public Command getAutonomousCommand() {
    chooserUpdate();
    return autoCommand.getSelected();
  }
  public Command getAutonomousWaitCommand() {
    return new WaitCommand(autoWait.getDouble(0));
  }

  public void loadCargoRotatorShuffleboard() {
    cargoTab.addNumber("Cargo Arm Angle", RobotContainer.m_cargoRotator::currentAngle);
    cargoTab.addBoolean("Cargo Rotator", RobotContainer.m_cargoRotator::isEnabled);
    cargoTab.addNumber("Cargo Arm Raw Angle", RobotContainer.m_cargoRotator::currentAngleRaw);
    cargoTab.addNumber("Cargo Rotator Setpoint", RobotContainer.m_cargoRotator::getSetpoint);

    cargoTab.add("Cargo Rotator PID", RobotContainer.m_cargoRotator.cargoRotatorPID);
  }
  public void loadCargoShooterShuffleboard() {
    cargoTab.addBoolean("Cargo Shooter", RobotContainer.m_cargoShooter::isEnabled);
    cargoTab.addNumber("Shooter Velocity", RobotContainer.m_cargoShooter::getVelocity);
        
    cargoTab.add("CargoShooterPID", RobotContainer.m_cargoShooter.cargoShooterPID);
  }
  public void loadCargoBeltShuffleboard(){
    cargoTab.addBoolean("Cargo Belt", RobotContainer.m_cargoBelt::isEnabled);
  }

  public void loadLimelightShuffleboard() {
    cargoTab.add("Alignment PID", AlignToUpperHub.alignPID);
    cargoTab.add("Ball Chase PID", ChaseBall.turnPID);

    cargoTab.addBoolean("Is Aligned To Hub", () -> AlignToUpperHub.isFinished);
    cargoTab.addNumber("Alignment offset", () -> AlignToUpperHub.offset);

    cargoTab.addNumber("Chase offset", () -> ChaseBall.offset);

    cargoTab.addNumber("Distance", () -> GetDistance.distance);
    cargoTab.addNumber("Optimal velocity", () -> GetDistance.optimalVelocity);
    cargoTab.addNumber("Optimal angle", () -> GetDistance.optimalStipeAngle);
  }

  public void loadClimbExtenderShuffleboard(ClimbExtender extender) {
    climbTab.addNumber(extender.getSide() + " Extension", extender::currentExtensionRaw);
    climbTab.addBoolean(extender.getSide() + " Extender", extender::isEnabled);
    
    climbTab.add(extender.getSide() + "Climb Extender PID", extender.extenderPID);
  }

  public void loadClimbRotatorShuffleboard(ClimbRotator rotator) {
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    climbTab.addNumber(rotator.getSide() + " Climb Rotator Angle", rotator::currentAngle);
    // a pop-up in shuffleboard that states if the rotator is on/off
    climbTab.addBoolean(rotator.getSide() + " Climb Rotator", rotator::isEnabled);

    climbTab.addNumber(rotator.getSide() + " Climb Rotator Goal", rotator::getSetPoint);
    
    // PID values that can be modified in shuffleboard
    climbTab.add(rotator.getSide() + " Climb Rotator PID", rotator.armPID);
  }

  public void loadBallDetectionShuffleboard(){
    cargoTab.addBoolean("Has Red Ball", RobotContainer.m_ballDetection::hasRedBall);
    cargoTab.addBoolean("Has Blue Ball", RobotContainer.m_ballDetection::hasBlueBall);
    cargoTab.addBoolean("Has Ball", RobotContainer.m_ballDetection::containsBall);
    cargoTab.addBoolean("Has Ball Securely", RobotContainer.m_ballDetection::containsBallSecurely);

  }

  private String getDriveModeString(){
    return Driver.getDriveMode().toString();
  }

  public boolean getLimelightRed(){
    return limelightColor.getBoolean(true);
  }
  public boolean getLimelightBlue(){
    return !limelightColor.getBoolean(true);
  }
}
  