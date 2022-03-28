package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class Log {

  DataLog m_log;
  int m_buffer = 100;
  int m_bufferStatus = 0;

  ArrayList<BooleanSupplier> m_booleanSuppliers = new ArrayList<BooleanSupplier>();
  ArrayList<BooleanLogEntry> m_booleanEntries = new ArrayList<BooleanLogEntry>();
  ArrayList<DoubleSupplier> m_doubleSuppliers = new ArrayList<DoubleSupplier>();
  ArrayList<DoubleLogEntry> m_doubleEntries = new ArrayList<DoubleLogEntry>();
  ArrayList<IntSupplier> m_intSuppliers = new ArrayList<IntSupplier>();
  ArrayList<IntegerLogEntry> m_intEntries = new ArrayList<IntegerLogEntry>();
  ArrayList<Supplier<String>> m_stringSuppliers = new ArrayList<Supplier<String>>();
  ArrayList<StringLogEntry> m_stringEntries = new ArrayList<StringLogEntry>();

  
  public Log(){
    DataLogManager.start();
    m_log = DataLogManager.getLog();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void initialize(){
    initializeDrivetrain(Robot.drive);

    initializeClimbRotator(Robot.rotatorL);
    initializeClimbRotator(Robot.rotatorR);
    initializeClimbExtender(Robot.extenderL);
    initializeClimbExtender(Robot.extenderR);

    initializeCargoRotator(Robot.arm);
    initializeCargoShooter(Robot.shooter);
    initializeCargoBelt(Robot.belt);
    initializeBallDetection(Robot.ballDetection);

    initializeCommandScheduler();
  }

  public void initializeClimbExtender(Extender extender){
    add(extender::isEnabled, "/climbExtender"+extender.getSide()+"/enabled");
    add(extender::currentExtensionRaw, "/climbExtender"+extender.getSide()+"/currentExtensionRaw");
    add(extender::reachedSetpoint, "/climbExtender"+extender.getSide()+"/reachedSetpoint");
  }
  public void initializeClimbRotator(Rotator rotator){
    add(rotator::isEnabled, "/climbRotator"+rotator.getSide()+"/enabled");
    add(rotator::reachedSetpoint, "/climbRotator"+rotator.getSide()+"/reachedSetpoint");
    add(rotator::currentAngle, "/climbRotator"+rotator.getSide()+"/currentAngle");
    add(rotator::currentAngleRaw, "/climbRotator"+rotator.getSide()+"/currentAngleRaw");
  }
  
  public void initializeCargoRotator(Arm rotator){
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
  public void initializeCargoShooter(Shooter shooter){
    add(shooter::isEnabled, "/cargoShooter/isEnabled");
    add(shooter::reachedSetpoint, "/cargoShooter/reachedSetpoint");
    add(shooter::getVelocity, "/cargoShooter/getVelocity");
  }
  public void initializeCargoBelt(Belt belt){
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
    StringLogEntry commandScheduler = new StringLogEntry(m_log, "/commandScheduler");
    CommandScheduler.getInstance().onCommandInitialize(command -> commandScheduler.append("Command initialized: " + command.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(command -> commandScheduler.append("Command interrupted: " + command.getName()));
    CommandScheduler.getInstance().onCommandFinish(command -> commandScheduler.append("Command finished: " + command.getName()));
  }

  public void add(BooleanSupplier supplier , String name){
    m_booleanSuppliers.add(supplier);
    m_booleanEntries.add(new BooleanLogEntry(m_log, name));
  }
  public void add(DoubleSupplier supplier , String name){
    m_doubleSuppliers.add(supplier);
    m_doubleEntries.add(new DoubleLogEntry(m_log, name));
  }
  public void add(IntSupplier supplier , String name){
    m_intSuppliers.add(supplier);
    m_intEntries.add(new IntegerLogEntry(m_log, name));
  }
  public void add(Supplier<String> supplier , String name){
    m_stringSuppliers.add(supplier);
    m_stringEntries.add(new StringLogEntry(m_log, name));
  }

  public void update(){
    for (int i = 0 ; i < m_booleanSuppliers.size() ; i++){
      m_booleanEntries.get(i).append(m_booleanSuppliers.get(i).getAsBoolean());
    }
    for (int i = 0 ; i < m_doubleSuppliers.size() ; i++){
      m_doubleEntries.get(i).append(m_doubleSuppliers.get(i).getAsDouble());
    }
    for (int i = 0 ; i < m_intSuppliers.size() ; i++){
      m_intEntries.get(i).append(m_intSuppliers.get(i).getAsInt());
    }
    for (int i = 0 ; i < m_stringSuppliers.size() ; i++){
      m_stringEntries.get(i).append(m_stringSuppliers.get(i).get());
    }
  }
  public void updateBuffer(){
    m_bufferStatus++;
    if (m_bufferStatus >= m_buffer){
      update();
      m_bufferStatus = 0;
    }
  }

}