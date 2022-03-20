package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.cargoCommands.AlignToUpperHub;
import frc.robot.commands.cargoCommands.GetDistance;
import frc.robot.commands.cargoCommands.Intake;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.commands.cargoCommands.Shoot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.robotConstants.shooterBelt.MarinusBeltConstants;
import frc.robot.robotConstants.shooterWheel.MarinusCargoShooterConstants;
import frc.robot.util.ClimberMethods;
import frc.robot.util.ShooterMethods;

public class Operator {

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  // these two are named a little weirdly because the command group for this needs to be at least a little readable
  public static MarinusClimbExtenderConstants extend = new MarinusClimbExtenderConstants();
  public static MarinusClimbRotatorConstants rotate = new MarinusClimbRotatorConstants();

  public static MarinusCargoRotatorConstants cargoConstants = new MarinusCargoRotatorConstants();
  public static MarinusBeltConstants beltConstants = new MarinusBeltConstants();
  public static MarinusCargoShooterConstants wheelConstants = new MarinusCargoShooterConstants();

  //operator buttons
  public static void configureButtonBindings() {
    shootBinds();
    // testShootBinds();
  }
  
  public static void shootBinds() {
    // Vision Shoot front
    controller.getButtons().Y().whenHeld(new Shoot(true, true, true));

    // Vision Shoot back
    controller.getButtons().A().whenHeld(new Shoot(true, true, false));

    // Manual Shoot front
    controller.getButtons().RT().whileActiveOnce(new Shoot(false, false, true, cargoConstants.kFrontOuttakeHighPos, wheelConstants.kFrontOuttakeHighSpeed));

    // Manual Shoot back
    controller.getButtons().RB().whenHeld(new Shoot(false, false, false, cargoConstants.kBackOuttakeHighPos, wheelConstants.kBackOuttakeHighSpeed));

    // Manual intake
    controller.getButtons().X().whenHeld(new Intake(cargoConstants.kUprightPos, false, Constants.kIsRedAlliance));
    controller.getButtons().X().whenReleased(new PositionArm(cargoConstants.kUprightPos));

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