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
