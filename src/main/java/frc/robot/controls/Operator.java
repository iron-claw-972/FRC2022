package frc.robot.controls;

import javax.xml.transform.Templates;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;


public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  static Joystick tempJoy = new Joystick(JoyConstants.kOperatorJoy);
  static JoystickButton tempButton = new JoystickButton(tempJoy, 1);
  

  //operator buttons
  public static void configureButtonBindings() {

    //wheel testing
    /**
    controller.getButtons().A().whenPressed(
        () -> RobotContainer.m_testWheel.enable());

    controller.getButtons().B().whenPressed(
        () -> RobotContainer.m_testWheel.setIntakeSpeed());
    controller.getButtons().X().whenPressed(
        () -> RobotContainer.m_testWheel.setStop());
    controller.getButtons().Y().whenPressed(
        () -> RobotContainer.m_testWheel.disable());
 */

    //arm testing
    controller.getButtons().A().whenPressed(
        () -> RobotContainer.m_testArm.enable());
    controller.getButtons().B().whenPressed(
        () -> RobotContainer.m_testArm.disable());

    controller.getButtons().Y().whenPressed(
        () -> RobotContainer.m_testArm.setOutput(0.1));
    controller.getButtons().X().whenPressed(
        () -> RobotContainer.m_testArm.setOutput(0));
      

  }

}
