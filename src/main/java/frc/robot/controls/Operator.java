package frc.robot.controls;

import javax.xml.transform.Templates;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;


public class Operator {

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));
  private int step;

  static Joystick tempJoy = new Joystick(JoyConstants.kOperatorJoy);
  static JoystickButton tempButton = new JoystickButton(tempJoy, 1);
  

  //operator buttons
  public static void configureButtonBindings() {
    // left trigger = extend extender
    // left bumper =  stop shooter, retract intake arm, lower extender, extends upwards slightly
    // another button = traverse a bar (angle the arm, extend, angle back slightly, compress extender)
    // start = resume
    // back = decrease step / e-stop
    
    controller.getButtons().LT().whenpressed(() -> RobotContainer.m_climbArm.set(90));
  }
}
