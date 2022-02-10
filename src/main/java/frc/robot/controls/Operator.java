package frc.robot.controls;

import javax.xml.transform.Templates;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.Arm;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  static Joystick tempJoy = new Joystick(JoyConstants.kOperatorJoy);
  static JoystickButton tempButton = new JoystickButton(tempJoy, 1);
  

  //operator buttons
  public static void configureButtonBindings() {
    
    
    controller.getButtons().B().whenPressed(
        () -> RobotContainer.m_arm.setRaw(0));
    controller.getButtons().A().whenPressed(
        () -> RobotContainer.m_arm.setRaw(-0.7));
    controller.getButtons().Y().whenPressed(
        () -> RobotContainer.m_arm.setRaw(0.7));


  }

}
