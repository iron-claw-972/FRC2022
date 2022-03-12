package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Log {

  DataLog log;

  ArrayList<DoubleSupplier> doubleSuppliers = new ArrayList<DoubleSupplier>();
  ArrayList<DoubleLogEntry> doubleEntries = new ArrayList<DoubleLogEntry>();
  ArrayList<IntSupplier> intSuppliers = new ArrayList<IntSupplier>();
  ArrayList<IntegerLogEntry> intEntries = new ArrayList<IntegerLogEntry>();
  ArrayList<BooleanSupplier> booleanSuppliers = new ArrayList<BooleanSupplier>();
  ArrayList<BooleanLogEntry> booleanEntries = new ArrayList<BooleanLogEntry>();
  
  public Log(){
    DataLogManager.start();
    log = DataLogManager.getLog();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void initialize(){
    
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
    
  }
  public void initializeCargoBelt(CargoBelt belt){
    
  }

  public void add(DoubleSupplier supplier , String name){
    doubleSuppliers.add(supplier);
    doubleEntries.add(new DoubleLogEntry(log, name));
  }
  public void add(IntSupplier supplier , String name){
    intSuppliers.add(supplier);
    intEntries.add(new IntegerLogEntry(log, name));
  }
  public void add(BooleanSupplier supplier , String name){
    booleanSuppliers.add(supplier);
    booleanEntries.add(new BooleanLogEntry(log, name));
  }

  public void update(){
    for (int i = 0 ; i < doubleSuppliers.size() ; i++){
      doubleEntries.get(i).append(doubleSuppliers.get(i).getAsDouble());
    }
    for (int i = 0 ; i < intSuppliers.size() ; i++){
      intEntries.get(i).append(intSuppliers.get(i).getAsInt());
    }
    for (int i = 0 ; i < booleanSuppliers.size() ; i++){
      booleanEntries.get(i).append(booleanSuppliers.get(i).getAsBoolean());
    }
  }
  
}
