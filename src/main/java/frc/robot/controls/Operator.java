package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.setup.controllers.*;

public class Operator{

  public static GameC controler = new GameC(new Joystick(JoyConstants.kOperatorJoy));

  //operator buttons
  public static void configureButtonBindings() {
      // controler.Button.B().whenPressed(() -> modSensitivity());
      // controler.Button.A().whenPressed(() -> modDrive());
    }

  }