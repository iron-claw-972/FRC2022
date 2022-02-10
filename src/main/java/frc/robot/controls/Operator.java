package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

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
      () -> RobotContainer.m_testArm.set(90));
  controller.getButtons().X().whenPressed(
        () -> RobotContainer.m_testArm.set(0));
  controller.getButtons().Y().whenPressed(
      () -> RobotContainer.m_testArm.disable());


  }

}
