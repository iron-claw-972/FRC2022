// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;   

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import com.ctre.phoenix.motorcontrol.can.*;
/** Add your docs here. */

public class PDPTesting extends SubsystemBase {

PowerDistribution PD = new PowerDistribution(0, ModuleType.kCTRE);
TalonSRX leftMotor1 = new TalonSRX(1); //14
double voltage;
double current;
double power;
double energy;
double leftMotor1Current;

@Override
public void periodic() {
    voltage = PD.getVoltage();
    current = PD.getTotalCurrent();
    power = PD.getTotalPower();
    energy = PD.getTotalEnergy();
    leftMotor1Current = PD.getCurrent(14);
    System.out.println(leftMotor1Current);
    leftMotor1.set(ControlMode.PercentOutput, 0.2)

}


}