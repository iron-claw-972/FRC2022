package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.robotConstants.extenderArm.*;
import frc.robot.robotConstants.climbArm.*;
import frc.robot.util.ClimbMethods;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  public static TraversoExtenderArmConstants excon = new TraversoExtenderArmConstants();
  public static TraversoClimbArmConstants clcon = new TraversoClimbArmConstants();

  static Joystick tempJoy = new Joystick(JoyConstants.kOperatorJoy);
  static JoystickButton tempButton = new JoystickButton(tempJoy, 1);
  

  // operator buttons
  public static void configureButtonBindings() {

    // extender goes up a little bit for the driver to go hook onto the mid-bar
    controller.getButtons().A().whenPressed(new SequentialCommandGroup(
      // the extender goes up a small amount
      new InstantCommand(() -> ClimbMethods.extenderHardExtend(excon.kSlightlyUpward))
      .andThen(
        // wait until it reaches its setpoint
        new WaitUntilCommand(ClimbMethods::extenderSetCheck)
      )
    ));

    // this extends the arm to its lowest point, extends the arm upwards a little,
    // the arm rotates backwards to its maximum, and the arm extends to its maxium
    controller.getButtons().B().whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> ClimbMethods.extenderHardExtend(excon.kMaxDownwards))
      .andThen(
        // wait until it reaches it setpoint
        new WaitUntilCommand(ClimbMethods::extenderSetCheck)
      ),
      // extend slightly upwards
      new InstantCommand(() -> ClimbMethods.extenderHardExtend(excon.kSlightlyUpward))
      .andThen(
        new WaitUntilCommand(ClimbMethods::extenderSetCheck)
      ),
      // rotate to the maximum backwards
      new InstantCommand(() -> ClimbMethods.rotatorHardAngle(clcon.kMaxBackward))
      .andThen(
        // wait until both reach their setpoints
        new WaitUntilCommand(ClimbMethods::rotatorSetCheck)
      ),
      // extender goes to its maximum point
      new InstantCommand(() -> ClimbMethods.extenderHardExtend(excon.kMaxUpwards))
      .andThen(
        // wait until the extender reaches its maximum point
        new WaitUntilCommand(ClimbMethods::extenderSetCheck)
      )
    ));

    // this rotates the arm to the next bar, straightens the arm to 90 degrees while also compressing
    controller.getButtons().X().whenPressed(new SequentialCommandGroup(
      // rotate the arm to the bar
      new InstantCommand(()-> ClimbMethods.rotatorHardAngle(clcon.kToBar))
      .andThen(
        // wait until the rotator reaches it setpoint
        new WaitUntilCommand(ClimbMethods::rotatorSetCheck)
      ),
      // extender goes to its lowest point
      new InstantCommand(() -> ClimbMethods.extenderHardExtend(excon.kMaxDownwards)),
      // rotator goes to 90 degrees
      new InstantCommand(() -> ClimbMethods.rotatorHardAngle(clcon.kNinetyDeg))
      .andThen(
        // wait until both reach their setpoints
        new WaitUntilCommand(ClimbMethods::extenderSetCheck),
        new WaitUntilCommand(ClimbMethods::rotatorSetCheck)
      )
    ));
  }
}
