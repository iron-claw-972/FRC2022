package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;

public class Operator{

  public static GameC controller = new GameC(new Joystick(JoyConstants.kOperatorJoy));

  //operator buttons
  public static void configureButtonBindings() {
      // controller.Button.B().whenPressed(() -> modSensitivity());
      // controller.Button.A().whenPressed(() -> modDrive());
    }

  }
