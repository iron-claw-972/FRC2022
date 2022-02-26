package frc.robot.controls;



import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.commands.Intake;
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
    // climbBinds();
    shootBinds();
  }

  public static void climbBinds() {
    controller.getDPad().up().whenPressed(new SequentialCommandGroup(
      // move the cargo arm to stow
      new PositionArm(cargoConstants.kStowPos),
      // extend upwards, go to 90 degrees
      new ClimberMove(extend.kMaxUpwards, rotate.kNinetyDeg)
    ));

    controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
      // extend downwards, remain 90 degrees
      new ClimberMove(extend.kMaxDownwards, rotate.kNinetyDeg),

      // by now, the static hooks should be on the bar

      // extend slightly upward, remain 90 degrees
      new ClimberMove(extend.kSlightlyUpward, rotate.kNinetyDeg)
    ));

    controller.getDPad().right().whenPressed(new SequentialCommandGroup(
      // go upwards and rotate backwards
      new ClimberMove(extend.kMaxUpwards, rotate.kMaxBackward),
      // remain going upwards and rotate towards the bar
      new ClimberMove(extend.kMaxUpwards, rotate.kToBar),
      // compress and rotate to 90 degrees
      new ClimberMove(extend.kMaxDownwards, rotate.kNinetyDeg)
    ));

    // resume the sequence
    controller.getButtons().START().whenPressed((new ParallelCommandGroup(
      new InstantCommand(() -> ClimberMethods.enableAll())
    )));

    // pause the sequence
    controller.getButtons().BACK().whenPressed((new ParallelCommandGroup(
      new InstantCommand(() -> ClimberMethods.disableAll())
    )));
  }

  public static void shootBinds() {
    controller.getButtons().RB().whenPressed(new ConditionalCommand(
      // shoot at a desired angle and outtake/intake speeds
      new Shoot(cargoConstants.kFrontOuttakeFarPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeFarSpeed, beltConstants.kOuttakeSpeed, true),
      new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, wheelConstants.kBackOuttakeFarSpeed, beltConstants.kOuttakeSpeed, true),
      ShooterMethods::isArmFront
    ));

    controller.getButtons().RT().whenActive(new ConditionalCommand(
      // shoot at a desired angle and outtake/intake speeds
      new Shoot(cargoConstants.kFrontOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false),
      new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kBackOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false),
      ShooterMethods::isArmFront
    ));

    // move arm to back
    controller.getButtons().A().whenPressed(new PositionArm(cargoConstants.kBackOuttakeFarPos));
    // move arm to front
    controller.getButtons().B().whenPressed(new PositionArm(cargoConstants.kFrontOuttakeFarPos));
    // intake
    controller.getButtons().X().whenPressed(new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kFrontOuttakeFarPos, false, Constants.kIsRedAlliance)); 
    // stow arm
    controller.getButtons().Y().whenPressed(new PositionArm(cargoConstants.kStowPos));
  }
}