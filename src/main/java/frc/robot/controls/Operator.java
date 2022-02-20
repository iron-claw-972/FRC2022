package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.GetDistance;

public class Operator {

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  // operator buttons
  public static void configureButtonBindings() {
    controller.getButtons().A().whileHeld(new GetDistance(RobotContainer.m_limelight));
    controller.getButtons().Y().whileHeld(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));
  }
}
