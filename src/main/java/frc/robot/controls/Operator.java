package frc.robot.controls;

import javax.xml.transform.Templates;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));
  

  //operator buttons
  public static void configureButtonBindings() {
    

  }
}
