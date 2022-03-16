package frc.robot.util;

import java.nio.Buffer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Log {

  DataLog log;
  int buffer = 100;
  int bufferStatus = 0;

  ArrayList<BooleanSupplier> booleanSuppliers = new ArrayList<BooleanSupplier>();
  ArrayList<BooleanLogEntry> booleanEntries = new ArrayList<BooleanLogEntry>();
  ArrayList<DoubleSupplier> doubleSuppliers = new ArrayList<DoubleSupplier>();
  ArrayList<DoubleLogEntry> doubleEntries = new ArrayList<DoubleLogEntry>();
  ArrayList<IntSupplier> intSuppliers = new ArrayList<IntSupplier>();
  ArrayList<IntegerLogEntry> intEntries = new ArrayList<IntegerLogEntry>();
  ArrayList<Supplier<String>> stringSuppliers = new ArrayList<Supplier<String>>();
  ArrayList<StringLogEntry> stringEntries = new ArrayList<StringLogEntry>();

  
  public Log(){
    DataLogManager.start();
    log = DataLogManager.getLog();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void initialize(){
    initializeDrivetrain(RobotContainer.m_drive);

    initializeClimbRotator(RobotContainer.m_climbRotatorL);
    initializeClimbRotator(RobotContainer.m_climbRotatorL);
    initializeClimbExtender(RobotContainer.m_extenderL);
    initializeClimbExtender(RobotContainer.m_extenderL);

    initializeCargoRotator(RobotContainer.m_cargoRotator);
    initializeCargoShooter(RobotContainer.m_cargoShooter);
    initializeCargoBelt(RobotContainer.m_cargoBelt);
    initializeBallDetection(RobotContainer.m_ballDetection);

    initializeCommandScheduler();
  }

  public void initializeClimbExtender(ClimbExtender extender){
    add(extender::isEnabled, "/climbExtender"+extender.getSide()+"/enabled");
    add(extender::currentExtensionRaw, "/climbExtender"+extender.getSide()+"/currentExtensionRaw");
    add(extender::reachedSetpoint, "/climbExtender"+extender.getSide()+"/reachedSetpoint");
  }
  public void initializeClimbRotator(ClimbRotator rotator){
    add(rotator::isEnabled, "/climbRotator"+rotator.getSide()+"/enabled");
    add(rotator::reachedSetpoint, "/climbRotator"+rotator.getSide()+"/reachedSetpoint");
    add(rotator::currentAngle, "/climbRotator"+rotator.getSide()+"/currentAngle");
    add(rotator::currentAngleRaw, "/climbRotator"+rotator.getSide()+"/currentAngleRaw");
  }
  
  public void initializeCargoRotator(CargoRotator rotator){
    add(rotator::isEnabled, "/cargoRotator/isEnabled");
    add(rotator::currentAngle, "/cargoRotator/currentAngle");
    add(rotator::currentAngleRaw, "/cargoRotator/currentAngleRaw");
    add(rotator::getSetpoint, "/cargoRotator/getSetpoint");
    add(rotator::isBackOutakeFar, "/cargoRotator/isBackOutakeFar");
    add(rotator::isBackOutakeNear, "/cargoRotator/isBackOutakeNear");
    add(rotator::isFront, "/cargoRotator/isFront");
    add(rotator::isFrontOutakeFar, "/cargoRotator/isFrontOutakeFar");
    add(rotator::isFrontOutakeNear, "/cargoRotator/isFrontOutakeNear");
    add(rotator::isIntake, "/cargoRotator/isIntake");
    add(rotator::isStow, "/cargoRotator/isStow");
    add(rotator::reachedSetpoint, "/cargoRotator/reachedSetpoint");
  }
  public void initializeCargoShooter(CargoShooter shooter){
    add(shooter::isEnabled, "/cargoShooter/isEnabled");
    add(shooter::reachedSetpoint, "/cargoShooter/reachedSetpoint");
    add(shooter::getVelocity, "/cargoShooter/getVelocity");
  }
  public void initializeCargoBelt(CargoBelt belt){
    add(belt::isEnabled, "/cargoBelt/isEnabled");
    
  }
  
  public void initializeBallDetection(BallDetection ballDetection){
    add(ballDetection::containsBall, "/ballDetection/containsBall");
    add(ballDetection::containsBallSecurely, "/ballDetection/containsBallSecurely");
    add(ballDetection::hasBlueBall, "/ballDetection/hasBlueBall");
    add(ballDetection::hasBlueBallSecure, "/ballDetection/hasBlueBallSecure");
    add(ballDetection::hasRedBall, "/ballDetection/hasRedBall");
    add(ballDetection::hasRedBallSecure, "/ballDetection/hasRedBallSecure");
  }
  public void initializeDrivetrain(Drivetrain drivetrain){
    add(drivetrain::getAverageEncoderDistance, "/drivetrain/getAverageEncoderDistance");
    add(drivetrain::getHeading, "/drivetrain/getHeading");
    add(drivetrain::getLeftPosition, "/drivetrain/getLeftPosition");
    add(drivetrain::getRightPosition, "/drivetrain/getRightPosition");
    add(drivetrain::getTurnRate, "/drivetrain/getTurnRate");
  }
  public void initializeCommandScheduler(){
    StringLogEntry commandScheduler = new StringLogEntry(log, "/commandScheduler");
    CommandScheduler.getInstance().onCommandInitialize(command -> commandScheduler.append("Command initialized: " + command.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(command -> commandScheduler.append("Command interrupted: " + command.getName()));
    CommandScheduler.getInstance().onCommandFinish(command -> commandScheduler.append("Command finished: " + command.getName()));
  }

  public void add(BooleanSupplier supplier , String name){
    booleanSuppliers.add(supplier);
    booleanEntries.add(new BooleanLogEntry(log, name));
  }
  public void add(DoubleSupplier supplier , String name){
    doubleSuppliers.add(supplier);
    doubleEntries.add(new DoubleLogEntry(log, name));
  }
  public void add(IntSupplier supplier , String name){
    intSuppliers.add(supplier);
    intEntries.add(new IntegerLogEntry(log, name));
  }
  public void add(Supplier<String> supplier , String name){
    stringSuppliers.add(supplier);
    stringEntries.add(new StringLogEntry(log, name));
  }

  public void update(){
    for (int i = 0 ; i < booleanSuppliers.size() ; i++){
      booleanEntries.get(i).append(booleanSuppliers.get(i).getAsBoolean());
    }
    for (int i = 0 ; i < doubleSuppliers.size() ; i++){
      doubleEntries.get(i).append(doubleSuppliers.get(i).getAsDouble());
    }
    for (int i = 0 ; i < intSuppliers.size() ; i++){
      intEntries.get(i).append(intSuppliers.get(i).getAsInt());
    }
    for (int i = 0 ; i < stringSuppliers.size() ; i++){
      stringEntries.get(i).append(stringSuppliers.get(i).get());
    }
  }
  public void updateBuffer(){
    bufferStatus++;
    if (bufferStatus >= buffer){
      update();
      bufferStatus = 0;
    }
  }

}