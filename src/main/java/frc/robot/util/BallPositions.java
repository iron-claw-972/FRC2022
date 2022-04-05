package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;

public enum BallPositions {
  //see stevens map for which cargos.
  R4(0, Alliance.Red), B1(1, Alliance.Blue), R5(3, Alliance.Red), B2(4, Alliance.Blue), B3(6, Alliance.Blue), R6(7, Alliance.Red), B4(8, Alliance.Blue), R1(9, Alliance.Red), B5(11, Alliance.Blue), R2(12, Alliance.Red), R3(14, Alliance.Red), B6(15, Alliance.Blue);

  public Translation2d m_pos;
  public Alliance m_alliance;
  public double m_angleAwayFromHub;

  // 0.850 Robot Width
  // 0.927 Robot Length
  
  private BallPositions(int rotIndex, Alliance alliance) {
    // 0 degrees is pointing right from the hub. So the centerline is 144 degrees. The first red ball is 11.25 degrees from the centerline then each ball is 22.5 degrees
    m_angleAwayFromHub = Units.degreesToRadians(114 + 11.25 + rotIndex*22.5);
    Translation2d ballPosRelativeToHub = new Translation2d(Math.cos(m_angleAwayFromHub) * Constants.field.distBallFromHub, Math.sin(m_angleAwayFromHub) * Constants.field.distBallFromHub);
    m_pos = Constants.field.hubPos.plus(ballPosRelativeToHub);
    m_alliance = alliance;
  }

  public Pose2d getRobotPoseFromBall() {
    Translation2d robotPosRelativeToHub = new Translation2d(Math.cos(m_angleAwayFromHub) * Constants.field.distRobotStartFromHub, Math.sin(m_angleAwayFromHub) * Constants.field.distRobotStartFromHub);
    return new Pose2d(Constants.field.hubPos.plus(robotPosRelativeToHub), new Rotation2d(m_angleAwayFromHub));
  }

  public Pose2d getRobotPoseFromBall(double dist) {
    Translation2d robotPosRelativeToHub = new Translation2d(Math.cos(m_angleAwayFromHub) * dist, Math.sin(m_angleAwayFromHub) * dist);
    return new Pose2d(Constants.field.hubPos.plus(robotPosRelativeToHub), new Rotation2d(m_angleAwayFromHub));
  }

  public static BallPositions getBall(int index, Alliance color) {
    switch (index) {
      case 1:
        return color == Alliance.Blue ? B1 : R1;
      case 2:
        return color == Alliance.Blue ? B2 : R2;
      case 3:
        return color == Alliance.Blue ? B3 : R3;
      case 4:
        return color == Alliance.Blue ? B4 : R4;
      case 5:
        return color == Alliance.Blue ? B5 : R5;
      case 6:
        return color == Alliance.Blue ? B6 : R6;
      default:
        return null;
    }
  }

  public void printBallPos() {
    System.out.println("Ball: " + m_pos.getX() + ", " + m_pos.getY());
    System.out.println("Robot: " + getRobotPoseFromBall().getX() + ", " + getRobotPoseFromBall().getY() + " angle: " + Units.radiansToDegrees(m_angleAwayFromHub));
  }
}
