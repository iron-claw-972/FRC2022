/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDriveSet extends CommandBase {
  private final Drivetrain m_drive;

  private double speed, rotation;

  public ArcadeDriveSet(Drivetrain subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //slew rate limiter removed temp need to move to controls
    // speed = Driver.getThrottleValue() * Constants.drive.kMaxSpeedMetersPerSecond;
    // rotation = Driver.getTurnValue() * Constants.drive.kMaxAngularSpeedRadiansPerSecond;

    // m_drive.feedForwardDrive(speed, rotation);

    m_drive.arcadeDrive(Driver.getThrottleValue(), Driver.getTurnValue());
    
  }
  
}
