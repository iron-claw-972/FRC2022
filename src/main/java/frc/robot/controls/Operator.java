package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.util.ClimberMethods;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  // these two are named a little weirdly because the command group for this needs to be at least a little readable
  public static TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();
  public static TraversoClimbRotatorConstants rotate = new TraversoClimbRotatorConstants();

  // constants for the arm rotator (yanis claw)
  public static TraversoCargoRotatorConstants cargoconstants = new TraversoCargoRotatorConstants();

  //operator buttons
  public static void configureButtonBindings() {
    climbBinds();
    
  }

  public static void climbBinds() {
    controller.getDPad().up().whenPressed(new SequentialCommandGroup(
      // arm rotates to 90 degrees
      new FunctionalCommand(
        ClimberMethods::enableRotator, // on init, do this
        () -> ClimberMethods.setAngle(rotate.kNinetyDeg), // on execute, do this
        interrupted -> ClimberMethods.disableRotator(), // on end, do this
        () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
        RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
      ),

      // extender goes all the way up
      new FunctionalCommand(
        // on initialization of the command, do:
        ClimberMethods::enableExtender, 
        // when executed, do:
        () -> ClimberMethods.setExtension(extend.kMaxUpwards),
        // when the command is ended, do:
        interrupted -> ClimberMethods.disableExtender(), 
        // when this is true, end
        () -> ClimberMethods.isExtenderAtSetpoint(), 
        // object requirements
        RobotContainer.m_extenderL, RobotContainer.m_extenderR
      )
    ));

    // this extends the arm to its lowest point and extends the arm upwards a little
    controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
      // extender goes to its lowest position
      new FunctionalCommand(
        ClimberMethods::enableExtender, // on init, do this
        () -> ClimberMethods.setExtension(extend.kMaxDownwards), // on execute, do this
        interrupted -> ClimberMethods.disableExtender(), // on end, do this
        () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
        RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
      ),

      // static hook should be hooked on now

      // extender goes up a little
      new FunctionalCommand(
        ClimberMethods::enableExtender, // on init, do this
        () -> ClimberMethods.setExtension(extend.kSlightlyUpward), // on execute, do this
        interrupted -> ClimberMethods.disableExtender(), // on end, do this
        () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
        RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
      )
    ));

    // the arm rotates backwards, extends, angles to the bar, compresses, && returns to 90 degrees
    controller.getDPad().right().whenPressed(new SequentialCommandGroup(

      // arm rotates max backwards
      new FunctionalCommand(
        ClimberMethods::enableRotator, // on init, do this
        () -> ClimberMethods.setAngle(rotate.kMaxBackward), // on execute, do this
        interrupted -> ClimberMethods.disableRotator(), // on end, do this
        () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
        RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
      ),

      // extender goes to its maximum point
      new FunctionalCommand(
        ClimberMethods::enableExtender, // on init, do this
        () -> ClimberMethods.setExtension(extend.kMaxUpwards), // on execute, do this
        interrupted -> ClimberMethods.disableExtender(), // on end, do this
        () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
        RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
      ),

      // rotate the arm to the bar
      new FunctionalCommand(
        ClimberMethods::enableRotator, // on init, do this
        () -> ClimberMethods.setAngle(rotate.kToBar), // on execute, do this
        interrupted -> ClimberMethods.disableRotator(), // on end, do this
        () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
        RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
      ),

      // run two commands at once
      new ParallelCommandGroup(
        // extender goes max downwards
        new FunctionalCommand(
          ClimberMethods::enableExtender, // on init, do this
          () -> ClimberMethods.setExtension(extend.kMaxDownwards), // on execute, do this
          interrupted -> ClimberMethods.disableExtender(), // on end, do this
          () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
          RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
        ),
        // arm rotates to 90 degrees
        new FunctionalCommand(
          ClimberMethods::enableRotator, // on init, do this
          () -> ClimberMethods.setAngle(rotate.kNinetyDeg), // on execute, do this
          interrupted -> ClimberMethods.disableRotator(), // on end, do this
          () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
          RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
        )
      )
    ));

    // resume the sequence
    controller.getButtons().START().whenPressed((new ParallelCommandGroup(
      new InstantCommand(() -> ClimberMethods.enableExtender()),
      new InstantCommand(() -> ClimberMethods.enableRotator())
    )));

    // end the sequence
    controller.getButtons().BACK().whenPressed((new ParallelCommandGroup(
      new InstantCommand(() -> ClimberMethods.disableExtender()),
      new InstantCommand(() -> ClimberMethods.disableRotator())
    )));
  }

  public static void armBinds() {
    controller.getButtons().RB().whenPressed(new SequentialCommandGroup(
    ));
  }
}
