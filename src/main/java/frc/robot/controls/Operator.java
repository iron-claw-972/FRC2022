package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.Rumble;
import frc.robot.commands.cargoCommands.AlignToUpperHub;
import frc.robot.commands.cargoCommands.GetDistance;
import frc.robot.commands.cargoCommands.Intake;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.commands.cargoCommands.Shoot;
import frc.robot.commands.climberCommands.ClimbExtenderMove;
import frc.robot.commands.climberCommands.ClimbMove;
import frc.robot.commands.climberCommands.ClimbRotatorMove;
import frc.robot.commands.climberCommands.ExtendDownwards;
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
    climbBindsHybrid();
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
    controller.getButtons().X()
      .whenHeld(new Intake(cargoConstants.kUprightPos, false, Constants.kIsRedAlliance))
      .whenReleased(new PositionArm(cargoConstants.kUprightPos));

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

  public static void climbBindsHybrid() {

    // when DPad Up is pressed, enable the extender and extend upwards to kMaxUpwards
    controller.getDPad().up().whenHeld(new ParallelCommandGroup(
      // stow the cargo subsystem
      new PositionArm(cargoConstants.kStowPos),
      new ClimbExtenderMove(extend.kRightMaxUpwards, extend.kLeftMaxUpwards)
    ));

    // when DPad Down is pressed, enable the extender and compress downwards to kMaxDownwards
    controller.getDPad().down().whenHeld(new ParallelCommandGroup(
      // stow the cargo subsystem
      new PositionArm(cargoConstants.kStowPos),
      new ExtendDownwards(extend.kAlwaysZero)
    ));

    // when DPad Right is pressed, enable the rotator and go to kMaxForward degrees
    controller.getDPad().right().whenPressed(new SequentialCommandGroup (
      new ClimbRotatorMove(rotate.kMaxForward)
    ));
    
    // when DPad Left is pressed, enable the rotator and go to kMaxBackward degrees
    controller.getDPad().left().whenPressed(new SequentialCommandGroup (
      new ClimbRotatorMove(rotate.kMaxBackward)
    ));

    // when nothing on the DPad is pressed, the extenders are disabled
    controller.getDPad().unpressed().whenPressed(
      new InstantCommand(() -> ClimberMethods.disableExtender())
    );

    // rotator goes to the bar
    controller.getButtons().LT().whenActive(new SequentialCommandGroup(
      new ClimbRotatorMove(rotate.kToBar)
    ));

    controller.getButtons().START().whenPressed(
      new ExtendDownwards(true)
    );

    controller.getButtons().LB().whenPressed(new SequentialCommandGroup(    
      new ClimbRotatorMove(rotate.kNinetyDeg),
      // when it reaches 90 degrees, compress
      new ExtendDownwards(extend.kAlwaysZero),
      
      // after the static hooks are on, extend slightly upwards
      new ClimbExtenderMove(extend.kRightSlightlyUpward, extend.kLeftSlightlyUpward),

      // extend fully and rotate backwards fully
      // rotator should theoretically be faster than the extender
      new ClimbRotatorMove(rotate.kMaxBackward),
      new PrintCommand("passed climb rotator move"),
      new ClimbExtenderMove(extend.kRightMaxUpwards, extend.kLeftMaxUpwards),

      // after we fully extend and rotate, rotate to the bar
      new ClimbRotatorMove(rotate.kToBar),

      // compress fully and rotate to 90 degrees
      new ParallelCommandGroup(
        new ExtendDownwards(extend.kAlwaysZero),
        new ClimbRotatorMove(rotate.kNinetyDeg)
      ),

      new ClimbExtenderMove(extend.kRightSlightlyUpward, extend.kLeftSlightlyUpward),

      // vibrate the controller
      new Rumble(controller)
    ));
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
      new ClimbExtenderMove(extend.kRightMaxUpwards, extend.kLeftMaxUpwards)
    ));

    // when DPad Down is pressed, enable the extender and compress downwards to kMaxDownwards
    controller.getDPad().down().whenHeld(new ParallelCommandGroup(
      // stow the cargo subsystem
      new PositionArm(cargoConstants.kStowPos),
      new ExtendDownwards(extend.kAlwaysZero)
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

    controller.getButtons().RB().whenPressed(
      new ExtendDownwards(true)
    );
  }


  public static void autoClimbBinds() {

    controller.getDPad().up().whenPressed(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          // stow the shooter
          new PositionArm(cargoConstants.kStowPos),

          // go to maximum extension, go to 90 degrees
          new ClimbMove(extend.kRightMaxUpwards, extend.kLeftMaxUpwards, rotate.kNinetyDeg)
        ),

        // vibrate the controller
        new Rumble(controller)
    ));

    controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
      new ClimbRotatorMove(rotate.kNinetyDeg),
      // when it reaches 90 degrees, compress
      new ExtendDownwards(extend.kAlwaysZero),
      
      // after the static hooks are on, extend slightly upwards
      new ClimbExtenderMove(extend.kRightSlightlyUpward, extend.kLeftSlightlyUpward),

      // vibrate the controller
      new Rumble(controller)
    ));

    controller.getDPad().right().whenPressed(new SequentialCommandGroup(
      // extend fully and rotate backwards fully
      // rotator should theoretically be faster than the extender
      new ClimbRotatorMove(rotate.kMaxBackward),
      new PrintCommand("passed climb rotator move"),
      new ClimbExtenderMove(extend.kRightMaxUpwards, extend.kLeftMaxUpwards),

      // after we fully extend and rotate, rotate to the bar
      new ClimbRotatorMove(rotate.kToBar)
    ));

    controller.getButtons().LB().whenPressed(new SequentialCommandGroup(
      // compress fully and rotate to 90 degrees
      new ParallelCommandGroup(
        new ExtendDownwards(extend.kAlwaysZero),
        new ClimbRotatorMove(rotate.kNinetyDeg)
      ),

      new ClimbExtenderMove(extend.kRightSlightlyUpward, extend.kLeftSlightlyUpward),

      // vibrate the controller
      new Rumble(controller)
    ));

    controller.getDPad().left().whenPressed(
      new ExtendDownwards(true)
    );

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