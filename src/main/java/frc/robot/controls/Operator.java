package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.Intake;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.GetDistance;
import frc.robot.commands.PositionArm;
import frc.robot.commands.Shoot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;

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
    // Shoot front
    controller.getButtons().Y().whenHeld(new Shoot(true, true, true));

    // Shoot back
    controller.getButtons().A().whenHeld(new Shoot(true, true, false));

    // Manual intake
    controller.getButtons().X().whenHeld(new Intake(cargoConstants.kBackLimelightScanPos, false, Constants.kIsRedAlliance));
    controller.getButtons().X().whenReleased(new PositionArm(cargoConstants.kFrontLimelightScanPos));

    // Stow arm
    controller.getButtons().B().whenPressed(new PositionArm(cargoConstants.kStowPos));
  }

  public static void testShootBinds() {
    // controller.getButtons().RT().whileActiveOnce(new Shoot(false, false, false, 175, -6000));
    // controller.getButtons().RB().whenHeld(new GetDistance(RobotContainer.m_limelight, RobotContainer.m_cargoRotator));

    // Align to upper hub front
    controller.getButtons().RT().whileActiveOnce(new SequentialCommandGroup(
      new PositionArm(cargoConstants.kFrontLimelightScanPos),
      new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive)
    ));

    // Align to upper hub back
    controller.getButtons().RB().whenHeld(new SequentialCommandGroup(
      new PositionArm(cargoConstants.kBackLimelightScanPos),
      new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive)
    ));
  }
}