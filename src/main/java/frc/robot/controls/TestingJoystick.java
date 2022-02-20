package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.util.ClimberMethods;

public class TestingJoystick {

  public static GameController controller = new GameController(new Joystick(3));

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


    controller.getDPad().up().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableExtender()),
      new InstantCommand(() -> ClimberMethods.setExtension(24))));
   

    controller.getDPad().down().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableExtender()),
      new InstantCommand(() -> ClimberMethods.setExtension(0))));
   

    controller.getDPad().right().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableRotator()),
      new InstantCommand(() -> ClimberMethods.setAngle(80))));
    
    
    controller.getDPad().left().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableRotator()),
      new InstantCommand(() -> ClimberMethods.setAngle(125))));
    

    controller.getDPad().upRight().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(24)),
     new InstantCommand(() -> ClimberMethods.setAngle(80)) 
    ));
    

    controller.getDPad().downRight().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(0)),
     new InstantCommand(() -> ClimberMethods.setAngle(80)) 
    ));
    

    controller.getDPad().upLeft().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(24)),
     new InstantCommand(() -> ClimberMethods.setAngle(125)) 
    ));
    

    controller.getDPad().downLeft().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(0)),
     new InstantCommand(() -> ClimberMethods.setAngle(125)) 
    ));
    

    controller.getDPad().unpressed().whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> ClimberMethods.disableExtender()),
      new InstantCommand(() -> ClimberMethods.disableRotator())));
  }
}
