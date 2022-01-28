package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.setup.controllers.*;

public class Operator{

  public static GameC controler = new GameC(new Joystick(JoyConstants.kOperatorJoy));

  public static double lowSensThrottle = 0.2 , lowSensTurn = 0.4, highSensThrottle = 1, highSensTurn = 0.5;
  public static double sensThrottle = lowSensThrottle, sensTurn = lowSensTurn;

  //operator buttons
  public static void configureButtonBindings() {
      // controler.Button.B().whenPressed(() -> modSensitivity());
      // controler.Button.A().whenPressed(() -> modDrive());
    }

  }