package frc.robot.controls;

import javax.xml.transform.Templates;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.robotConstants.extenderArm.*;
import frc.robot.robotConstants.climbArm.*;


public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  TraversoExtenderArmConstants excon = new TraversoExtenderArmConstants();
  TraversoClimbArmConstants clcon = new TraversoClimbArmConstants();

  //operator buttons
  public static void configureButtonBindings() {

    //wheel testing
    /*
    controller.getButtons().A().whenPressed(
        () -> RobotContainer.m_testWheel.enable());

    controller.getButtons().B().whenPressed(
        () -> RobotContainer.m_testWheel.setIntakeSpeed());
    controller.getButtons().X().whenPressed(
        () -> RobotContainer.m_testWheel.setStop());
    controller.getButtons().Y().whenPressed(
        () -> RobotContainer.m_testWheel.disable());
    */

    //arm testing

    controller.getButtons().A().whenPressed(
        () -> RobotContainer.m_testArm.enable());
    controller.getButtons().B().whenPressed(
        () -> RobotContainer.m_testArm.disable());

    controller.getButtons().Y().whenPressed(
        () -> RobotContainer.m_testArm.setOutput(0.1));
    // controller.getButtons().X().whenPressed(
    //     () -> RobotContainer.m_testArm.setOutput(0));

    controller.getButtons().X().whileHeld(
        () -> RobotContainer.m_testArm.setOutput(
        -controller.getJoystickAxis().leftY()));
    controller.getButtons().X().whenReleased
        (() -> RobotContainer.m_testArm.setOutput(0));
        
    controller.getButtons().RB().whenPressed(
        () -> RobotContainer.m_testArm.setEncoder(SmartDashboard.getNumber("set encoder", 0)));
    controller.getButtons().LB().whenPressed(
        () -> RobotContainer.m_testArm.setGoal(SmartDashboard.getNumber("goal", 0)));
          

  }
}
