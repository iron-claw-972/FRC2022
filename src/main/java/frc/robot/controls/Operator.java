package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.robotConstants.extenderArm.*;
import frc.robot.util.climbMethods;
import frc.robot.robotConstants.climbArm.*;


public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  public static TraversoExtenderArmConstants excon = new TraversoExtenderArmConstants();
  public static TraversoClimbArmConstants clcon = new TraversoClimbArmConstants();

  static Joystick tempJoy = new Joystick(JoyConstants.kOperatorJoy);
  static JoystickButton tempButton = new JoystickButton(tempJoy, 1);
  

  // operator buttons
  public static void configureButtonBindings() {

    // tl;dr:
    // extender goes up a little bit for the driver to go hook onto the mid-bar
    controller.getButtons().A().whenPressed(new SequentialCommandGroup(
      // the extender goes up a small amount
      new InstantCommand(() -> climbMethods.extenderHardExtend(excon.kSlightlyUpward))
      .andThen(
        // wait until it reaches its setpoint
        new WaitUntilCommand(climbMethods::extenderSetCheck)
      )
    ));

    // tl;dr:
    // this extends the arm to its lowest point, extends the arm upwards a little,
    // the arm rotates backwards to its maximum, and the arm extends to its maxium
    controller.getButtons().B().whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> climbMethods.extenderHardExtend(excon.kMaxDownwards))
      .andThen(
        // wait until it reaches it setpoint
        new WaitUntilCommand(climbMethods::extenderSetCheck)
      ),
      // extend slightly upwards
      new InstantCommand(() -> climbMethods.extenderHardExtend(excon.kSlightlyUpward))
      .andThen(
        new WaitUntilCommand(climbMethods::extenderSetCheck)
      ),
      // rotate to the maximum backwards
      new InstantCommand(() -> climbMethods.rotatorHardAngle(clcon.kMaxBackward))
      .andThen(
        // wait until both reach their setpoints
        new WaitUntilCommand(climbMethods::rotatorSetCheck)
      ),
      // extender goes to its maximum point
      new InstantCommand(() -> climbMethods.extenderHardExtend(excon.kMaxUpwards))
      .andThen(
        // wait until the extender reaches its maximum point
        new WaitUntilCommand(climbMethods::extenderSetCheck)
      )
    ));


    // tl;dr:
    // this rotates the arm to the next bar, straightens the arm to 90 degrees while also compressing
    controller.getButtons().X().whenPressed(new SequentialCommandGroup(
      // rotate the arm to the bar
      new InstantCommand(()-> climbMethods.rotatorHardAngle(clcon.kToBar))
      .andThen(
        // wait until the rotator reaches it setpoint
        new WaitUntilCommand(climbMethods::rotatorSetCheck)
      ),
      // extender goes to its lowest point
      new InstantCommand(() -> climbMethods.extenderHardExtend(excon.kMaxDownwards)),
      // rotator goes to 90 degrees
      new InstantCommand(() -> climbMethods.rotatorHardAngle(clcon.kNinetyDeg))
      .andThen(
        // wait until both reach their setpoints
        new WaitUntilCommand(climbMethods::extenderSetCheck),
        new WaitUntilCommand(climbMethods::rotatorSetCheck)
      )
    ));
  }
}
