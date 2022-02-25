/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive; 
  double setpoint, error;

  PIDController PID = new PIDController(0.2, 0, 0);

  public DriveDistance(double setpoint_, Drivetrain subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
    setpoint = setpoint_;
    PID.setTolerance(100);
    ;
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    setpoint = setpoint - m_drive.getLeftPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    error = setpoint - m_drive.getLeftPosition();
    m_drive.feedForwardDrive(PID.calculate(error), 0);
    
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.feedForwardDrive(0, 0);
      super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
      return PID.atSetpoint();
  }
  
}
