package frc.robot.autonomous.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import frc.robot.Constants;
import frc.robot.Constants.*;

public class Pathweaver {

    public static Drivetrain m_drive = new Drivetrain();
    public static Intake m_intake = new Intake();
    
    private static Trajectory autonomousTrajectory;
    private static RamseteCommand ramseteCommand = new RamseteCommand(
            autonomousTrajectory,
            m_drive::getPose,
            m_drive.getRamseteController(),
            m_drive.getFeedforward(),
            m_drive.getDriveKinematics(),
            m_drive::getWheelSpeeds,
            m_drive.getLeftRamsetePIDController(),
            m_drive.getRightRamsetePIDController(),
            // RamseteCommand passes volts to the callback
            m_drive::tankDriveVolts,
            m_drive);

    public void setup(){
        // Reset odometry to the starting pose of the trajectory.
        m_drive.resetOdometry(autonomousTrajectory.getInitialPose());
    }

    public static void loadAutonomousTrajectory(String trajectoryName) {
        String trajectoryJSON = "paths/output/" + trajectoryName + ".wpilib.json";
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          autonomousTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          DriverStation.reportWarning(
              "Unable to open trajectory: " + trajectoryJSON + "\n" +
                  "Falling back to default trajectory",
              ex.getStackTrace());
    
          // Create a voltage constraint to ensure we don't accelerate too fast
          var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            m_drive.getFeedforward(),
            m_drive.getDriveKinematics(),
            Constants.kMaxVoltage);
    
          // Create config for trajectory
          TrajectoryConfig config = new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                  // Add kinematics to ensure max speed is actually obeyed
                  .setKinematics(m_drive.getDriveKinematics())
                  // Apply the voltage constraint
                  .addConstraint(autoVoltageConstraint);
              
          // Fallback to default trajectory
          autonomousTrajectory = TrajectoryGenerator.generateTrajectory(
              // Make a square and end where we started
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(new Translation2d(3, 0), new Translation2d(3, 3), new Translation2d(0, 3)),
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass config
              config);
        }
    }
    
    //returnes auto command group
    public static ParallelDeadlineGroup pathweaverCommand(){
        return new ParallelDeadlineGroup(
        ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0)), 
        new RunCommand(() -> m_intake.run(0.5)));
    }
} 