package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.PositionArm;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.util.ClimberMethods;

public class ManualClimb {

  public static GameController controller = new GameController(new Joystick(3));

  public static DonkClimbExtenderConstants extend = new DonkClimbExtenderConstants();
  public static DonkClimbRotatorConstants rotate = new DonkClimbRotatorConstants();

  public static void configureButtonBindings() {

    controller.getDPad().right().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.disableRotator()),
      new InstantCommand(() -> ClimberMethods.setRotatorOutput(0.2))));
    
    
    controller.getDPad().left().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.disableRotator()),
      new InstantCommand(() -> ClimberMethods.setRotatorOutput(-0.2))));


    controller.getDPad().up().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableExtender()),
      new InstantCommand(() -> ClimberMethods.setExtension(extend.kMaxUpwards))));
   

    controller.getDPad().down().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableExtender()),
      new InstantCommand(() -> ClimberMethods.setExtension(extend.kMaxDownwards))));

    controller.getButtons().A().whenPressed(
      new PositionArm(170)
    );
   /*

    controller.getDPad().right().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableRotator()),
      new InstantCommand(() -> ClimberMethods.setAngle(rotate.kMaxForward))));
    
    
    controller.getDPad().left().whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimberMethods.enableRotator()),
      new InstantCommand(() -> ClimberMethods.setAngle(rotate.kMaxBackward))));
    

    controller.getDPad().upRight().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(extend.kMaxUpwards)),
     new InstantCommand(() -> ClimberMethods.setAngle(rotate.kMaxForward))
    ));
    

    controller.getDPad().downRight().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(0)),
     new InstantCommand(() -> ClimberMethods.setAngle(rotate.kMaxForward)) 
    ));
    

    controller.getDPad().upLeft().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(extend.kMaxUpwards)),
     new InstantCommand(() -> ClimberMethods.setAngle(rotate.kMaxBackward)) 
    ));
    

    controller.getDPad().downLeft().whenPressed(new SequentialCommandGroup(
     new InstantCommand(() -> ClimberMethods.enableExtender()),
     new InstantCommand(() -> ClimberMethods.enableRotator()), 
     new InstantCommand(() -> ClimberMethods.setExtension(extend.kMaxDownwards)),
     new InstantCommand(() -> ClimberMethods.setAngle(rotate.kMaxBackward)) 
    ));
    
    */
    controller.getDPad().unpressed().whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> ClimberMethods.setRotatorOutput(0)),
      new InstantCommand(() -> ClimberMethods.disableExtender()),
      new InstantCommand(() -> ClimberMethods.disableRotator())));

    SmartDashboard.putNumber("set angle", 90);
    controller.getButtons().X().whenPressed(new SequentialCommandGroup(
    new InstantCommand(() -> ClimberMethods.enableRotator()), 
    new InstantCommand(() -> ClimberMethods.setAngle(SmartDashboard.getNumber("set angle", 90))) 
    ));
  }

}
