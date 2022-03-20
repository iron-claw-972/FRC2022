package frc.robot.controls;

import controllers.GameController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.JoyConstants;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.commands.climberCommands.ClimbExtenderMove;
import frc.robot.commands.climberCommands.ClimbRotatorMove;
import frc.robot.commands.climberCommands.ClimberMove;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.MarinusClimbExtenderConstants;
import frc.robot.robotConstants.climbRotator.MarinusClimbRotatorConstants;
import frc.robot.util.ClimberMethods;

public class ClimbOperator {
  
  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorClimbJoy));

  // these two are named a little weirdly because the command group for this needs to be at least a little readable
  public static MarinusClimbExtenderConstants extend = new MarinusClimbExtenderConstants();
  public static MarinusClimbRotatorConstants rotate = new MarinusClimbRotatorConstants();

  public static MarinusCargoRotatorConstants cargoConstants = new MarinusCargoRotatorConstants();

  //operator buttons
  public static void configureButtonBindings() {
    climbBinds();
  }

  public static void climbBinds() {

    controller.getButtons().leftJoystickPressedButton().whenPressed(
      new InstantCommand(ClimberMethods::removeLimiter)
    );

    controller.getButtons().rightJoystickPressedButton().whenPressed(
      new InstantCommand(ClimberMethods::enableLimiter)
    );

    // when DPad Up is pressed, enable the extender and extend upwards to kMaxUpwards
    controller.getDPad().up().whenHeld(new ParallelCommandGroup(
      // stow the cargo subsystem
      new PositionArm(cargoConstants.kStowPos),
      new ClimbExtenderMove(extend.kMaxUpwards)
    ));

    // when DPad Down is pressed, enable the extender and compress downwards to kMaxDownwards
    controller.getDPad().down().whenHeld(new ParallelCommandGroup(
      // stow the cargo subsystem
      new PositionArm(cargoConstants.kStowPos),
      new ClimbExtenderMove(extend.kMaxDownwards)
    ));

    // when DPad Right is pressed, enable the rotator and go to kMaxForward degrees
    controller.getDPad().right().whenPressed(new SequentialCommandGroup (
      new ClimbRotatorMove(rotate.kMaxForward)
    ));
    
    // when DPad Left is pressed, enable the rotator and go to kMaxBackward degrees
    controller.getDPad().left().whenPressed(new SequentialCommandGroup (
      new ClimbRotatorMove(rotate.kMaxBackward)
    ));

    // when LB is pressed, enable the rotator and go to kNinetyDeg degrees
    controller.getButtons().LB().whenPressed(new SequentialCommandGroup(
      new ClimbRotatorMove(rotate.kNinetyDeg)
    ));

    // when nothing on the DPad is pressed, the extenders are disabled
    controller.getDPad().unpressed().whenPressed(
      new InstantCommand(() -> ClimberMethods.disableExtender())
    );

    // rotator goes to the bar
    controller.getButtons().LT().whenActive(new SequentialCommandGroup(
      new ClimbRotatorMove(rotate.kToBar)
    ));
  }

  public static void autoClimbBinds() {

    controller.getDPad().up().whenPressed(new ParallelCommandGroup(
      // move the cargo arm to stow
      new PositionArm(cargoConstants.kStowPos),

      // extend upwards, go an angle where we can hook the static hook
      new ClimberMove(extend.kMaxUpwards, rotate.kMaxForward)
    ));

    controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
      // extend downwards, go to 90 degrees
      new ClimbExtenderMove(extend.kMaxDownwards),
      new ClimbRotatorMove(rotate.kNinetyDeg),

      // by now, the static hooks should be on the bar

      // extend slightly upward, remain 90 degrees
      new ClimberMove(extend.kSlightlyUpward, rotate.kNinetyDeg)
    ));

    controller.getDPad().right().whenPressed(new SequentialCommandGroup(
      // go upwards and rotate backwards
      new ClimberMove(extend.kMaxUpwards, rotate.kMaxBackward),
      // remain going upwards and rotate towards the bar
      new ClimbRotatorMove(rotate.kToBar),
      // compress and rotate to 90 degrees
      new ClimberMove(extend.kMaxDownwards, rotate.kNinetyDeg)
    ));

    // resume the sequence
    controller.getButtons().START().whenPressed(
      new InstantCommand(() -> ClimberMethods.enableAll()
    ));

    // pause the sequence
    controller.getButtons().BACK().whenPressed(
      new InstantCommand(() -> ClimberMethods.disableAll()
    ));
  }

  }
