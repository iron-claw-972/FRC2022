package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.Intake;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.ClimbExtenderMove;
import frc.robot.commands.ClimbRotatorMove;
import frc.robot.commands.ClimberMove;
import frc.robot.commands.GetDistance;
import frc.robot.commands.PositionArm;
import frc.robot.commands.Shoot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;
import frc.robot.util.ClimberMethods;
import frc.robot.util.ShooterMethods;

public class Operator {

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  // these two are named a little weirdly because the command group for this needs to be at least a little readable
  public static TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();
  public static TraversoClimbRotatorConstants rotate = new TraversoClimbRotatorConstants();

  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();

  //operator buttons
  public static void configureButtonBindings() {
    shootBinds();
    testShootBinds();
  }
  
  public static void shootBinds() {
    // move arm to back
    controller.getButtons().Y().whenHeld(new Shoot(true, true, true));

    // move arm to front
    controller.getButtons().A().whenHeld(new Shoot(true, true, false));

    controller.getButtons().X().whenHeld(new Intake(cargoConstants.kBackLimelightScanPos, false, Constants.kIsRedAlliance));
    controller.getButtons().X().whenReleased(new PositionArm(cargoConstants.kBackLimelightScanPos));

    controller.getButtons().B().whenPressed(new PositionArm(cargoConstants.kStowPos));
  }

  public static void testShootBinds() {
    controller.getButtons().RB().whileHeld(new SequentialCommandGroup(
      // new PositionArm(cargoConstants.kBackLimelightScanPos).withTimeout(0.7),
      new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive)
    ));

    controller.getButtons().RT().whileActiveOnce(new GetDistance(RobotContainer.m_limelight));
  }
}