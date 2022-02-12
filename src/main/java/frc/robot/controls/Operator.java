package frc.robot.controls;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.*;

public class Operator {

  public static Joystick controller = new Joystick(0);
  public static final JoystickButton buttonA = new JoystickButton(controller, 1);
  public static final JoystickButton buttonB = new JoystickButton(controller, 2);
  public static final JoystickButton buttonX = new JoystickButton(controller, 3);

  public static ShooterBelt belt = new ShooterBelt();
  public static ShooterWheels wheels = new ShooterWheels();

  // operator buttons
  public static void configureButtonBindings() {
    buttonA.whenPressed(() -> {
      wheels.setSpeed(0.5);
    }).whenReleased(() -> {
      wheels.stop();
    });

    buttonB.whenPressed(() -> {
      wheels.setIntakeSpeed();
    });

    buttonX.whenPressed(() -> {
      wheels.setOutakeSpeed();
    });
  }

}
