package frc.robot.controls;

import controllers.GameController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.JoyConstants;
import frc.robot.commands.ClimbExtenderMove;
import frc.robot.commands.ClimbRotatorMove;
import frc.robot.commands.ClimberMove;
import frc.robot.commands.PositionArm;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;
import frc.robot.robotConstants.climbRotator.TraversoClimbRotatorConstants;
import frc.robot.util.ClimberMethods;

public class ClimbOperator {
  
  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorClimbJoy));

  // these two are named a little weirdly because the command group for this needs to be at least a little readable
  public static TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();
  public static TraversoClimbRotatorConstants rotate = new TraversoClimbRotatorConstants();

  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();

  //operator buttons
  public static void configureButtonBindings() {
    climbBinds();
  }

  public static void climbBinds() {

    controller.getButtons().leftJoyButton().whenPressed(
      new InstantCommand(ClimberMethods::removeLimiter)
    );

    controller.getButtons().rightJoyButton().whenPressed(
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
      // stow the shooter
      new PositionArm(cargoConstants.kStowPos),

      // go to maximum extension, go to 120 degrees
      new ClimberMove(extend.kMaxUpwards, rotate.kMaxBackward)
    ));

    controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
      // go to 90 degrees
      new ClimbRotatorMove(rotate.kNinetyDeg),

      // when it reaches 110 degrees, compress
      new ClimbExtenderMove(extend.kMaxDownwards),

      // when it compresses fully, go to ninety degrees
      new ClimbRotatorMove(rotate.kNinetyDeg),
      
      // after the static hooks are on, extend slightly upwards
      new ClimbExtenderMove(extend.kSlightlyUpward)
    ));

    controller.getDPad().right().whenPressed(new SequentialCommandGroup(
      // extend fully and rotate backwards fully
      // rotator should theoretically be faster than the extender
      new ClimberMove(extend.kMaxUpwards, rotate.kMaxBackward),

      // after we fully extend and rotate, rotate to the bar
      new ClimbRotatorMove(rotate.kToBar),

      // compress fully and rotate to 90 degrees
      new ClimberMove(extend.kMaxDownwards, rotate.kNinetyDeg)
    ));

    // resume the climbing sequence
    controller.getButtons().START().whenPressed(
      new InstantCommand(() -> ClimberMethods.enableAll()
    ));

    // pause the climbing sequence (press START to reenable the sequence)
    controller.getButtons().BACK().whenPressed(
      new InstantCommand(() -> ClimberMethods.disableAll()
    ));
  }
}
