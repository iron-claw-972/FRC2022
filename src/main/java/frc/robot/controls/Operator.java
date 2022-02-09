package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.Arm;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  //operator buttons
  public static void configureButtonBindings() {
    controller.getButtons().B().whenPressed(
        () -> RobotContainer.m_arm.setRaw(0));
    controller.getButtons().A().whenPressed(
        () -> RobotContainer.m_arm.setRaw(-0.01));
    controller.getButtons().Y().whenPressed(
        () -> RobotContainer.m_arm.setRaw(0.01));
  }



}
