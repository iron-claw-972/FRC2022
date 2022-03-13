package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.Intake;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.ClimbExtenderMove;
import frc.robot.commands.ClimbRotatorMove;
import frc.robot.commands.ClimberMove;
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
  }
  
  public static void shootBinds() {
    // controller.getButtons().RB().whenHeld(new ConditionalCommand(
    //   new Shoot(cargoConstants.kFrontOuttakeFarPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeFarSpeed, beltConstants.kOuttakeSpeed, false, 0),
    //   new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, wheelConstants.kBackOuttakeFarSpeed, beltConstants.kOuttakeSpeed, false, 0),
    //   ShooterMethods::isArmFront
    // ));

    // controller.getButtons().X().whenHeld(new Shoot(90, beltConstants.kIntakeSpeed, 0, beltConstants.kOuttakeSpeed, false, 0));


    // controller.getButtons().RT().whenActive(new ConditionalCommand(
    //   // shoot at a desired angle and outtake/intake speeds
    //   new Shoot(cargoConstants.kFrontOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false, 0),
    //   new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kBackOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false, 0),
    //   ShooterMethods::isArmFront
    // ));


    // move arm to back
    controller.getButtons().Y().whenPressed(new Shoot(true, true, true));

    // move arm to front
    controller.getButtons().A().whenPressed(new Shoot(true, true, false));

    controller.getButtons().X().whenHeld(new Intake(cargoConstants.kBackOuttakeFarPos, false, Constants.kIsRedAlliance));

    controller.getButtons().B().whenPressed(new PositionArm(cargoConstants.kStowPos));
    // controller.getButtons().X().whenReleased(new SequentialCommandGroup(
    //   new PositionArm(cargoConstants.kFrontOuttakeHighPos),
    //   new InstantCommand(() -> ShooterMethods.disableShiitake()),
    //   new InstantCommand(() -> RobotContainer.m_cargoRotator.resetPID())
    // ));

    // controller.getButtons().Y().whenPressed(new PositionArm(cargoConstants.kStowPos));
    // controller.getButtons().Y().whenHeld(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));
    // controller.getButtons().Y().whenHeld(new SequentialCommandGroup(
    //   new InstantCommand(() -> ShooterMethods.enableBelt()),
    //   new InstantCommand(() -> ShooterMethods.setBeltPower(beltConstants.kIntakeSpeed)))
    // );
    // controller.getButtons().Y().whenReleased(new InstantCommand(() -> ShooterMethods.disableBelt()));

    // controller.getButtons().LB().whenHeld(new SequentialCommandGroup(
    //   new InstantCommand(() -> ShooterMethods.enableBelt()),
    //   new InstantCommand(() -> ShooterMethods.setBeltPower(beltConstants.kOuttakeSpeed)))
    // );
    // controller.getButtons().LB().whenReleased(new InstantCommand(() -> ShooterMethods.disableBelt()));

    // controller.getButtons().RB().whenPressed(new GetDistance(RobotContainer.m_limelight, RobotContainer.m_cargoRotator));
  }
}