package frc.robot.controls;


import frc.robot.Robot;
import frc.robot.commands.Rumble;
import frc.robot.commands.cargo.*;
import frc.robot.commands.climb.*;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.CargoUtil;
import frc.robot.util.ClimbUtil;
import lib.controllers.*;
import lib.controllers.GameController.GCButton;
import lib.controllers.GameController.DPad;

public class Operator {

  public static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  //operator buttons
  public static void configureControls() {
    if (!DriverStation.isJoystickConnected(Constants.oi.kOperatorJoy)) {
      // Don't try to configure bindings if controller not plugged in
      DriverStation.reportWarning("Operator controller not connected to Port " + Constants.oi.kOperatorJoy, true);
      return;
    }

    configureCargoControls();
    // configureCargoTestControls();
    configureClimbControls();
  }
  
  private static void configureCargoControls() {
    // Vision Shoot front
    operator.get(GCButton.Y).whenHeld(new SequentialCommandGroup(
      new InstantCommand(() -> Robot.drive.tankDriveVolts(0, 0)),
      new Shoot(true, true, true)
    ));

    // Vision Shoot back
    operator.get(GCButton.A).whenHeld(new SequentialCommandGroup(
      new PrintCommand("A Pressed"),
      new InstantCommand(() -> Robot.drive.tankDriveVolts(0, 0)),
      new Shoot(true, true, false)
    ));

    // Manual Shoot front
    operator.get(operator.RIGHT_TRIGGER_BUTTON).whileActiveOnce(new Shoot(false, false, true, Constants.arm.kFrontOuttakeHighPos, Constants.shooter.kFrontOuttakeHighSpeed));

    // Manual Shoot back
    operator.get(GCButton.RB).whenHeld(new Shoot(false, false, false, Constants.arm.kBackOuttakeHighPos, Constants.shooter.kBackOuttakeHighSpeed));

    // Manual intake
    operator.get(GCButton.X)
      .whenHeld(new Intake(Constants.arm.kUprightPos, false))
      .whenReleased(new PositionArm(Constants.arm.kUprightPos).andThen(() -> Robot.ll.setUpperHubPipeline()));

    // Stow arm
    operator.get(GCButton.B).whenPressed(new PositionArm(Constants.arm.kStowPos));
    // operator.get(Button.B).whenHeld(new AlignToUpperHub());
  }

  public static void configureCargoTestControls() {
    // controller.get().RT().whileActiveOnce(new Shoot(false, false, false, 175, -6000));
    // controller.get().RB().whenHeld(new GetDistance(Robot.m_limelight, Robot.mArm));

    // operator.get(Button.Y).whenHeld(new SequentialCommandGroup(
    //   new InstantCommand(() -> CargoUtil.setAngle(CargoUtil::getTestArmAngle)),
    //   new InstantCommand(() -> CargoUtil.enableArm()),
    //   new WaitUntilCommand(() -> CargoUtil.isArmAtSetpoint()),
    //   new InstantCommand(() -> CargoUtil.setWheelRPM(CargoUtil::getTestShooterSpeed)),
    //   new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
    //   new InstantCommand(() -> CargoUtil.enableBelt()),
    //   new InstantCommand(() -> CargoUtil.enableWheel()),
    //   // new WaitCommand(1),
    //   new WaitUntilCommand(() -> CargoUtil.isWheelAtSetpoint()),
    //   new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kOuttakeSpeed)),
    //   new WaitCommand(0.4),
    //   new InstantCommand(() -> CargoUtil.disableShiitake())
    // ));
    // operator.get(Button.Y).whenReleased(new InstantCommand(() -> CargoUtil.disableShiitake()));


    // operator.get(Button.A).whenHeld(new SequentialCommandGroup(
    //   new InstantCommand(() -> CargoUtil.setAngle(CargoUtil::getTestArmAngle)),
    //   new InstantCommand(() -> CargoUtil.enableArm()),
    //   new WaitUntilCommand(() -> CargoUtil.isArmAtSetpoint()),
    //   new InstantCommand(() -> CargoUtil.setWheelRPM(CargoUtil::getTestShooterSpeed)),
    //   new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
    //   new InstantCommand(() -> CargoUtil.enableWheel()),
    //   new InstantCommand(() -> CargoUtil.enableBelt()),
    //   new WaitCommand(999999)
    //   // new WaitUntilCommand(() -> CargoUtil.isWheelAtSetpoint()),
    // ));
    // operator.get(Button.A).whenReleased(new InstantCommand(() -> CargoUtil.disableShiitake()));

    operator.get(GCButton.Y).whenHeld(new GetDistance(Robot.ll, Robot.arm));

    operator.get(GCButton.B).whenHeld(new SequentialCommandGroup(
      new InstantCommand(() -> CargoUtil.setAngle(CargoUtil::getTestArmAngle)),
      new InstantCommand(() -> CargoUtil.enableArm()),
      new WaitUntilCommand(() -> CargoUtil.isArmAtSetpoint())
    ));
    // operator.get(Button.B).whenReleased(new InstantCommand(() -> CargoUtil.disableArm()));

    // Align to upper hub front
    // controller.get().RT().whileActiveOnce(new SequentialCommandGroup(
    //   new PositionArm(Constants.arm.kFrontLimelightScanPos),
    //   new AlignToUpperHub(Robot.m_limelight, Robot.m_drive)
    // ));

    // // Align to upper hub back
    // controller.get().RB().whenHeld(new SequentialCommandGroup(
    //   new PositionArm(Constants.arm.kBackLimelightScanPos),
    //   new AlignToUpperHub(Robot.m_limelight, Robot.m_drive)
    // ));
  }

  private static void configureClimbControls() {

    // Pressing down on the left joystick enables manual control
    operator.get(GCButton.LEFT_JOY).whenPressed(
      new InstantCommand(ClimbUtil::removeLimiter)
    );

    // Pressing down on the right joystick disables manual control
    operator.get(GCButton.RIGHT_JOY).whenPressed(
      new InstantCommand(ClimbUtil::enableLimiter)
    );

    // when DPad Up is pressed, enable the extender and extend upwards to kMaxUpwards
    operator.get(DPad.UP).whenHeld(new ParallelCommandGroup(
      // stow the cargo subsystem
      new PositionArm(Constants.arm.kStowPos),
      new Extend(SmartDashboard.getNumber("Left Max Extension", Constants.extender.kLeftMaxUpwards), SmartDashboard.getNumber("Right Max Extension", Constants.extender.kRightMaxUpwards))
    ));

    // when DPad Down is pressed, enable the extender and compress downwards to kMaxDownwards
    operator.get(DPad.DOWN).whenHeld(new ParallelCommandGroup(
      // stow the cargo subsystem
      new PositionArm(Constants.arm.kStowPos),
      new Retract(Constants.extender.kAlwaysZero)
    ));

    // when DPad Right is pressed, enable the rotator and go to kMaxForward degrees
    operator.get(DPad.RIGHT).whenPressed(new SequentialCommandGroup (
      new Rotate(Constants.rotator.kMaxForwardL, Constants.rotator.kMaxForwardR)
    ));
    
    // when DPad Left is pressed, enable the rotator and go to kMaxBackward degrees
    operator.get(DPad.LEFT).whenPressed(new SequentialCommandGroup (
      new Rotate(Constants.rotator.kMaxBackwardL, Constants.rotator.kMaxBackwardR)
    ));

    // when nothing on the DPad is pressed, the extenders are disabled
    operator.get(DPad.UNPRESSED).whenPressed(
      new InstantCommand(() -> ClimbUtil.disableExtender())
    );

    // rotator goes to the bar
    operator.get(operator.LEFT_TRIGGER_BUTTON).whenActive(new SequentialCommandGroup(
      new Rotate(Constants.rotator.kToBarL, Constants.rotator.kToBarR)
    ));

    operator.get(GCButton.START).whenPressed(
      new Retract(true)
    );

    operator.get(GCButton.LB).whenPressed(new SequentialCommandGroup(    
      new Rotate(Constants.rotator.kMaxForwardL, Constants.rotator.kMaxForwardR),
      // when it reaches 90 degrees, compress
      new Retract(Constants.extender.kAlwaysZero),
      
      // after the static hooks are on, extend slightly upwards
      new Extend(Constants.extender.kLeftSlightlyUpward, Constants.extender.kRightSlightlyUpward),

      // extend fully and rotate backwards fully
      // rotator should theoretically be faster than the extender
      new Rotate(Constants.rotator.kMaxBackwardL, Constants.rotator.kMaxBackwardR),
      new PrintCommand("passed climb rotator move"),
      new Extend(Constants.extender.kLeftMaxUpwards, Constants.extender.kRightMaxUpwards),

      // after we fully extend and rotate, rotate to the bar
      new Rotate(Constants.rotator.kToBarL, Constants.rotator.kToBarR),

      // compress fully and rotate to 90 degrees
      new ParallelCommandGroup(
        new Retract(Constants.extender.kAlwaysZero),
        new Rotate(Constants.rotator.kMaxForwardL, Constants.rotator.kMaxForwardR)
      ),

      new Extend(Constants.extender.kLeftSlightlyUpward, Constants.extender.kRightSlightlyUpward),

      // vibrate the controller
      new Rumble(operator)
    ));
  }

  // public static void climbBinds() {

  //   operator.get(Button.LEFT_JOY).whenPressed(
  //     new InstantCommand(ClimberMethods::removeLimiter)
  //   );

  //   operator.get(Button.RIGHT_JOY).whenPressed(
  //     new InstantCommand(ClimberMethods::enableLimiter)
  //   );

  //   // when DPad Up is pressed, enable the extender and extend upwards to kMaxUpwards
  //   operator.get(DPad.UP).whenHeld(new ParallelCommandGroup(
  //     // stow the cargo subsystem
  //     new PositionArm(Constants.arm.kStowPos),
  //     new ClimbExtenderMove(Constants.extender.kRightMaxUpwards, Constants.extender.kLeftMaxUpwards)
  //   ));

  //   // when DPad Down is pressed, enable the extender and compress downwards to kMaxDownwards
  //   operator.get(DPad.DOWN).whenHeld(new ParallelCommandGroup(
  //     // stow the cargo subsystem
  //     new PositionArm(Constants.arm.kStowPos),
  //     new ExtendDownwards(Constants.extender.kAlwaysZero)
  //   ));

  //   // when DPad Right is pressed, enable the rotator and go to kMaxForward degrees
  //   operator.get(DPad.RIGHT).whenPressed(new SequentialCommandGroup (
  //     new ClimbRotatorMove(Constants.rotator.kMaxForward)
  //   ));
    
  //   // when DPad Left is pressed, enable the rotator and go to kMaxBackward degrees
  //   operator.get(DPad.LEFT).whenPressed(new SequentialCommandGroup (
  //     new ClimbRotatorMove(Constants.rotator.kMaxBackward)
  //   ));

  //   // when LB is pressed, enable the rotator and go to kMaxForward degrees
  //   operator.get(Button.LB).whenPressed(new SequentialCommandGroup(
  //     new ClimbRotatorMove(Constants.rotator.kMaxForward)
  //   ));

  //   // when nothing on the DPad is pressed, the extenders are disabled
  //   operator.get(DPad.UNPRESSED).whenPressed(
  //     new InstantCommand(() -> ClimberMethods.disableExtender())
  //   );

  //   // rotator goes to the bar
  //   operator.get(operator.LEFT_TRIGGER_BUTTON).whenActive(new SequentialCommandGroup(
  //     new ClimbRotatorMove(Constants.rotator.kToBar)
  //   ));

  //   operator.get(Button.RB).whenPressed(
  //     new ExtendDownwards(true)
  //   );
  // }


  // public static void autoClimbBinds() {

  //   operator.get(DPad.UP).whenPressed(
  //     new SequentialCommandGroup(
  //       new ParallelCommandGroup(
  //         // stow the shooter
  //         new PositionArm(Constants.arm.kStowPos),

  //         // go to maximum extension, go to 90 degrees
  //         new ClimbMove(Constants.extender.kRightMaxUpwards, Constants.extender.kLeftMaxUpwards, Constants.rotator.kMaxForward)
  //       ),

  //       // vibrate the controller
  //       new Rumble(operator)
  //   ));

  //   operator.get(DPad.DOWN).whenPressed(new SequentialCommandGroup(    
  //     new ClimbRotatorMove(Constants.rotator.kMaxForward),
  //     // when it reaches 90 degrees, compress
  //     new ExtendDownwards(Constants.extender.kAlwaysZero),
      
  //     // after the static hooks are on, extend slightly upwards
  //     new ClimbExtenderMove(Constants.extender.kRightSlightlyUpward, Constants.extender.kLeftSlightlyUpward),

  //     // vibrate the controller
  //     new Rumble(operator)
  //   ));

  //   operator.get(DPad.RIGHT).whenPressed(new SequentialCommandGroup(
  //     // extend fully and rotate backwards fully
  //     // rotator should theoretically be faster than the extender
  //     new ClimbRotatorMove(Constants.rotator.kMaxBackward),
  //     new PrintCommand("passed climb rotator move"),
  //     new ClimbExtenderMove(Constants.extender.kRightMaxUpwards, Constants.extender.kLeftMaxUpwards),

  //     // after we fully extend and rotate, rotate to the bar
  //     new ClimbRotatorMove(Constants.rotator.kToBar)
  //   ));

  //   operator.get(Button.LB).whenPressed(new SequentialCommandGroup(
  //     // compress fully and rotate to 90 degrees
  //     new ParallelCommandGroup(
  //       new ExtendDownwards(Constants.extender.kAlwaysZero),
  //       new ClimbRotatorMove(Constants.rotator.kMaxForward)
  //     ),

  //     new ClimbExtenderMove(Constants.extender.kRightSlightlyUpward, Constants.extender.kLeftSlightlyUpward),

  //     // vibrate the controller
  //     new Rumble(operator)
  //   ));

  //   operator.get(DPad.LEFT).whenPressed(
  //     new ExtendDownwards(true)
  //   );

  //   // resume the climbing sequence
  //   operator.get(Button.START).whenPressed(
  //     new InstantCommand(() -> ClimberMethods.enableAll()
  //   ));

  //   // pause the climbing sequence (press START to reenable the sequence)
  //   operator.get(Button.BACK).whenPressed(
  //     new InstantCommand(() -> ClimberMethods.disableAll()
  //   ));
  // }
}