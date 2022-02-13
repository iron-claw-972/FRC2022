package frc.robot.controls;

import javax.xml.transform.Templates;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.robotConstants.extenderArm.*;
import frc.robot.robotConstants.climbArm.*;


public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  TraversoExtenderArmConstants excon = new TraversoExtenderArmConstants();
  TraversoClimbArmConstants clcon = new TraversoClimbArmConstants();

  static Joystick tempJoy = new Joystick(JoyConstants.kOperatorJoy);
  static JoystickButton tempButton = new JoystickButton(tempJoy, 1);
  

  //operator buttons
  public static void configureButtonBindings() {
    // left trigger = extend extender
    // left bumper =  stop shooter, retract intake arm, lower extender, extends upwards slightly
    // another button = traverse a bar (angle the arm, extend, angle back slightly, compress extender)W
    // start = resume
    // back = decrease step / e-stop
  }
}
