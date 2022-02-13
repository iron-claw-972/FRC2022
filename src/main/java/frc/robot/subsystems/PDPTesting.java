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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.*;
/** Add your docs here. */

public class PDPTesting extends SubsystemBase {

PowerDistribution PD = new PowerDistribution(0, ModuleType.kCTRE);
TalonFX leftMotor1 = new TalonFX(24); //14
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
    leftMotor1Current = PD.getCurrent(0);
    // System.out.println(leftMotor1Current);
    leftMotor1.set(ControlMode.PercentOutput, 0.2);
    if (true) {
        SmartDashboard.putNumber("current0", PD.getCurrent(0));
        SmartDashboard.putNumber("current1", PD.getCurrent(1));
        SmartDashboard.putNumber("current2", PD.getCurrent(2));
        SmartDashboard.putNumber("current3", PD.getCurrent(3));
        SmartDashboard.putNumber("current4", PD.getCurrent(4));
        SmartDashboard.putNumber("current5", PD.getCurrent(5));
        SmartDashboard.putNumber("current6", PD.getCurrent(6));
        SmartDashboard.putNumber("current7", PD.getCurrent(7));
        SmartDashboard.putNumber("current8", PD.getCurrent(8));
        SmartDashboard.putNumber("current9", PD.getCurrent(9));
        SmartDashboard.putNumber("current10", PD.getCurrent(10));
        SmartDashboard.putNumber("current11", PD.getCurrent(11));
        SmartDashboard.putNumber("current12", PD.getCurrent(12));
        SmartDashboard.putNumber("current13", PD.getCurrent(13));
        SmartDashboard.putNumber("current14", PD.getCurrent(14));
        SmartDashboard.putNumber("current15", PD.getCurrent(15));
    }
    SmartDashboard.putNumber("total Volatage", PD.getVoltage());
}


}