package frc.robot.controls;

import java.time.Instant;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.util.ClimberMethods;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  public static TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();
  public static TraversoClimbRotatorConstants rotate = new TraversoClimbRotatorConstants();

  //operator buttons
  public static void configureButtonBindings() {
    
    //arm testing

    // controller.getButtons().A().whenPressed(
    //     () -> RobotContainer.m_rotatorR.enable());
    // controller.getButtons().B().whenPressed(
    //     () -> RobotContainer.m_rotatorR.disable());

    // controller.getButtons().Y().whenPressed(
    //     () -> RobotContainer.m_rotatorR.setOutput(0.1));
    // // controller.getButtons().X().whenPressed(
    // //     () -> RobotContainer.m_testArm.setOutput(0));

    // controller.getButtons().X().whileHeld(
    //     () -> RobotContainer.m_rotatorR.setOutput(
    //     controller.getJoystickAxis().leftY()));
    // controller.getButtons().X().whenReleased
    //     (() -> RobotContainer.m_rotatorR.setOutput(0));
        
    // controller.getButtons().RB().whenPressed(
    //     () -> RobotContainer.m_rotatorR.setEncoder(SmartDashboard.getNumber("set encoder", 0)));
    // controller.getButtons().LB().whenPressed(
    //     () -> RobotContainer.m_rotatorR.setGoal(SmartDashboard.getNumber("goal", 0)));


    controller.getDPad().up().whenPressed(new SequentialCommandGroup(
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
      ),

      // as an extra precaution
      new InstantCommand(() -> ClimberMethods.disableExtender()),
      new InstantCommand(() -> ClimberMethods.disableRotator())
    ));
  }
}
