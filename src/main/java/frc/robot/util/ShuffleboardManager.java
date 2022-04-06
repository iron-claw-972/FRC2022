package frc.robot.util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.routines.OneBallAuto;
import frc.robot.commands.auto.routines.Tar2FourBall;
import frc.robot.commands.auto.routines.TwoBallAuto;
import frc.robot.commands.auto.routines.Tar2ThreeBall;
import frc.robot.commands.auto.routines.Vision3BallAuto;
import frc.robot.commands.cargo.*;
import frc.robot.constants.Constants;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_cargoTab = Shuffleboard.getTab("Cargo");
  ShuffleboardTab m_climbTab = Shuffleboard.getTab("Climb");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry m_autoWait = m_autoTab.add("Auto Wait", 0.0).getEntry();
  NetworkTableEntry m_isFar = m_autoTab.add("Flexible Auto First Shoot is Fender", true).getEntry();
  NetworkTableEntry m_distance = m_autoTab.add("Flexible Auto Drive Distance", 0.0).getEntry();
  NetworkTableEntry m_shootSecond = m_autoTab.add("Flexible Auto Shoot a Second Time", true).getEntry();
  NetworkTableEntry m_intakeSecond = m_autoTab.add("Flexible Auto Get and Shoot a Second Time", true).getEntry();
  NetworkTableEntry m_limelightColor = m_cargoTab.add("Limelight (Red)", true).getEntry();

  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();
  
  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    m_mainTab.addBoolean("Is Teleop", DriverStation::isTeleop);
    m_mainTab.addNumber("left drive encoder", Robot.drive::getLeftPosition);
    m_autoTab.addNumber("Navx Position", Robot.drive::getHeading);

    m_autoTab.addNumber("X", Robot.drive::getPoseX);
    m_autoTab.addNumber("Y", Robot.drive::getPoseX);
    m_autoTab.addNumber("Rotation", Robot.drive::getPoseRotation);
    
    SmartDashboard.putNumber("auto rot", 100);

    SmartDashboard.putData(Robot.drive.m_field);

    // climbTab.addNumber("Max Extension Ticks", () -> extenderConstants.kExtenderMaxArmTicks);
    chooserUpdate();
    subsystemSpam();
    time();
    update();

    m_autoTab.add("Auto Chooser", m_autoCommand);
  }

  public void update() {
    
  }

  public void time() {
    m_mainTab.addNumber("Time Left", DriverStation::getMatchTime);
    m_mainTab.addNumber("Time Left Until Endgame" , this::getDriverStationTimeTillEndGame);
    // primaryTab.add("Auto Wait", 0);
  }
  private double getDriverStationTimeTillEndGame(){
    return DriverStation.getMatchTime() -30;
  }

  public void chooserUpdate() {
    // originally 0.8492
    
    m_autoCommand.addOption("1 Ball Auto", new OneBallAuto());

    m_autoCommand.addOption("2 Ball Auto", new TwoBallAuto());

    m_autoCommand.addOption("3 Ball Auto", new Tar2ThreeBall(Alliance.Blue));

    m_autoCommand.addOption("4 Ball Auto", new Tar2FourBall(Alliance.Blue));

    m_autoCommand.addOption("DoNothing - there be dragons past here", new DoNothing());

    m_autoCommand.addOption("RedVision3Ball", new Vision3BallAuto(true));
    m_autoCommand.addOption("BlueVision3Ball", new Vision3BallAuto(false));

    m_autoCommand.addOption("RedTar2ThreeBall", new Tar2ThreeBall(Alliance.Red));

    m_autoCommand.addOption("RedTar2FourBall", new Tar2FourBall(Alliance.Red));

    m_autoCommand.addOption("Rotation", new DriveRotation(SmartDashboard.getNumber("auto rot", 100)));

    m_autoCommand.addOption("TestPath", 
      new InstantCommand(() -> Robot.drive.resetOdometry(BallPositions.B3.getRobotPoseFromBall())).andThen(
      new PathweaverCommand("Test2Ball", Robot.drive))
    );

    m_autoCommand.addOption("Reset Pose Start", 
    new InstantCommand(() -> Robot.drive.resetOdometry(BallPositions.B3.getRobotPoseFromBall())).andThen(new PrintCommand("Robot: " + BallPositions.B3.getRobotPoseFromBall().getX() + ", " + BallPositions.B3.getRobotPoseFromBall().getY()))
    );

    m_autoCommand.addOption("Reset Pose Zero", new InstantCommand(() -> Robot.drive.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

    m_autoCommand.addOption("Reset Pose Ball", new InstantCommand(() -> Robot.drive.resetOdometry(
      new Pose2d(
        BallPositions.B3.m_pos, 
        new Rotation2d(BallPositions.B3.m_angleAwayFromHub)
      ))).andThen(new PrintCommand("Ball: " + BallPositions.B3.m_pos.getX() + ", " + BallPositions.B3.m_pos.getY()))
    );
    
    m_autoCommand.addOption("Reset Pose Hub", new InstantCommand(() -> Robot.drive.resetOdometry(
      new Pose2d(
        Constants.field.hubPos, 
        new Rotation2d(BallPositions.B3.m_angleAwayFromHub)
      ))).andThen(new PrintCommand("Hub: " + Constants.field.hubPos.getX() + ", " + Constants.field.hubPos.getY()))
    );

  }
  public void subsystemSpam() {
    // put subsystem shuffleboard things in here!

    loadClimbExtenderShuffleboard(Robot.extenderL);
    loadClimbExtenderShuffleboard(Robot.extenderR);

    loadClimbRotatorShuffleboard(Robot.rotatorL);
    loadClimbRotatorShuffleboard(Robot.rotatorR);

    loadCargoShooterShuffleboard();
    loadCargoRotatorShuffleboard();
    loadCargoBeltShuffleboard();

    loadBallDetectionShuffleboard();
    loadLimelightShuffleboard();
    
    loadCommandSchedulerShuffleboard();

  }

  public Command getAutonomousCommand() {
    chooserUpdate();
    return m_autoCommand.getSelected();
  }
  public Command getAutonomousWaitCommand() {
    return new WaitCommand(m_autoWait.getDouble(0));
  }

  public void loadCargoRotatorShuffleboard() {
    m_cargoTab.addNumber("Cargo Arm Angle", Robot.arm::currentAngle);
    m_cargoTab.addBoolean("Cargo Rotator", Robot.arm::isEnabled);
    m_cargoTab.addNumber("Cargo Arm Raw Angle", Robot.arm::currentAngleRaw);
    m_cargoTab.addNumber("Cargo Rotator Setpoint", Robot.arm::getSetpoint);
    m_cargoTab.addBoolean("Cargo Arm at Setpoint", Robot.arm.m_armPID::atSetpoint);

    m_cargoTab.add("Cargo Rotator PID", Robot.arm.m_armPID);
  }
  public void loadCargoShooterShuffleboard() {
    m_cargoTab.addBoolean("Cargo Shooter", Robot.shooter::isEnabled);
    m_cargoTab.addNumber("Shooter Velocity", Robot.shooter::getVelocity);
        
    m_cargoTab.add("CargoShooterPID", Robot.shooter.m_shooterPID);

    m_cargoTab.addBoolean("Shooter at Setpoint", Robot.shooter::reachedSetpoint);

    SmartDashboard.putNumber("Shooter FF", Constants.shooter.kForward);
    SmartDashboard.putNumber("Shooter kS", Constants.shooter.kS);
    SmartDashboard.putNumber("Limelight angle factor", Constants.ll.kAngularFactor);
    m_cargoTab.add(Robot.drive.m_dDrive);

    SmartDashboard.putNumber("Test shooter speed", -2500);
    SmartDashboard.putNumber("Test arm angle", 108);
  }
  public void loadCargoBeltShuffleboard(){
    m_cargoTab.addBoolean("Cargo Belt", Robot.belt::isEnabled);
  }

  public void loadLimelightShuffleboard() {
    m_cargoTab.add("Alignment PID", AlignToUpperHub.alignPID);
    m_cargoTab.add("Ball Chase PID", ChaseBall.turnPID);

    m_cargoTab.addBoolean("Is Aligned To Hub", () -> AlignToUpperHub.isFinished);
    m_cargoTab.addNumber("Alignment offset (deg)", () -> AlignToUpperHub.offset);

    m_cargoTab.addNumber("Chase offset (deg)", () -> ChaseBall.offset);

    m_cargoTab.addNumber("Pivot Distance (in)", () -> Units.metersToInches(GetDistance.pivotDistance));
    m_cargoTab.addNumber("Limelight Distance (in)", () -> Units.metersToInches(GetDistance.limelightDistance));
    m_cargoTab.addNumber("Optimal velocity (ft)", () -> GetDistance.optimalVelocity);
    // cargoTab.addNumber("Optimal RPM", () -> ShooterMethods.velocityToRPM(() -> GetDistance.optimalVelocity));
    m_cargoTab.addNumber("Optimal stipe angle (deg)", () -> GetDistance.optimalStipeAngle);
    m_cargoTab.addBoolean("getDistance Is Finished", () -> GetDistance.isFinished);

    m_cargoTab.addNumber("Tx (deg)", Robot.ll::getHubHorizontalAngularOffset);
    m_cargoTab.addNumber("Ty (deg)", Robot.ll::getVerticalAngularOffset);

    m_cargoTab.addNumber("Limelight latency (ms)", Robot.ll::getLatency);

    // SmartDashboard.putNumber("Front Shooting velocity", -2900);
    // SmartDashboard.putNumber("Front Stipe angle", 80);

    // SmartDashboard.putNumber("Back Shooting velocity", -2900);
    // SmartDashboard.putNumber("Back Stipe angle", 154);
    SmartDashboard.putNumber("Front Shot Efficiency", Constants.shooter.kFrontShotEfficiency);
    SmartDashboard.putNumber("Back Shot Efficiency", Constants.shooter.kBackShotEfficiency);

    SmartDashboard.putNumber("Front Distance Factor", Constants.ll.kFrontLimelightDistanceFactor);
    SmartDashboard.putNumber("Back Distance Factor", Constants.ll.kBackLimelightDistanceFactor);
  }

  public void loadClimbExtenderShuffleboard(Extender extender) {
    m_climbTab.addNumber(extender.getSide() + " Extension", extender::currentExtensionRaw);
    m_climbTab.addBoolean(extender.getSide() + " Extender", extender::isEnabled);
    
    m_climbTab.add(extender.getSide() + "Climb Extender PID", extender.m_extenderPID);
    m_climbTab.addBoolean(extender.getSide() + " Extender Setpoint", extender::reachedSetpoint);
    m_climbTab.addBoolean(extender.getSide() + " Limit Switch", extender::compressionLimitSwitch);
    m_climbTab.addBoolean(extender.getSide() + " Manual", extender::isManual);
  }

  public void loadClimbRotatorShuffleboard(Rotator rotator) {
    // a pop-up in shuffleboard that allows you to see how much the arm extended in inches
    m_climbTab.addNumber(rotator.getSide() + " Climb Rotator Angle", rotator::currentAngle);
    // a pop-up in shuffleboard that states if the rotator is on/off
    m_climbTab.addBoolean(rotator.getSide() + " Climb Rotator", rotator::isEnabled);

    m_climbTab.addNumber(rotator.getSide() + " Climb Rotator Goal", rotator::getGoal);
    
    // PID values that can be modified in shuffleboard
    m_climbTab.add(rotator.getSide() + " Climb Rotator PID", rotator.armPID);
    m_climbTab.addBoolean(rotator.getSide() + " Climb Rotator Setpoint Reached", rotator::reachedSetpoint);

    SmartDashboard.putNumber(rotator.getSide() + " Rotator FF", 0);
  }

  public void loadBallDetectionShuffleboard(){
    m_cargoTab.addBoolean("Has Red Ball", Robot.ballDetection::hasRedBall);
    m_cargoTab.addBoolean("Has Blue Ball", Robot.ballDetection::hasBlueBall);
    m_cargoTab.addBoolean("Has Ball", Robot.ballDetection::containsBall);
    m_cargoTab.addBoolean("Has Ball Securely", Robot.ballDetection::containsBallSecurely);

  }

  public boolean getLimelightRed(){
    return m_limelightColor.getBoolean(true);
  }
  public boolean getLimelightBlue(){
    return !m_limelightColor.getBoolean(true);
  }

  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString("Command initialized: " + command.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString("Command interrupted: " + command.getName()));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString("Command finished: " + command.getName()));
  }
}
  
