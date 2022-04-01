package frc.robot.controls;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.cargo.*;
import frc.robot.constants.Constants;
import frc.robot.util.ClimbUtil;
import lib.controllers.*;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class ManualClimb {

  public static GameController manualClimb = new GameController(Constants.oi.kManualClimbJoy);

  public static void configureControls() {
    if (!DriverStation.isJoystickConnected(Constants.oi.kManualClimbJoy)) {
      // Don't try to configure bindings if controller not plugged in
      DriverStation.reportWarning("Manual Climb controller not connected to Port " + Constants.oi.kManualClimbJoy, true);
      return;
    }

    configureManualClimbControls();
  }

  private static void configureManualClimbControls() {
    manualClimb.get(DPad.RIGHT).whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimbUtil.disableRotator()),
      new InstantCommand(() -> ClimbUtil.setRotatorOutput(0.2))));
    
    
    manualClimb.get(DPad.LEFT).whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimbUtil.disableRotator()),
      new InstantCommand(() -> ClimbUtil.setRotatorOutput(-0.2))));


    // STRING SHOULD BE UNWINDING
    manualClimb.get(DPad.UP).whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimbUtil.disableExtender()),
      new InstantCommand(() -> ClimbUtil.setExtenderOutput(.2))
    ));
   
    // STRING SHOULD BE WINDING
    manualClimb.get(DPad.DOWN).whenPressed(new SequentialCommandGroup (
      new InstantCommand(() -> ClimbUtil.disableExtender()),
      new InstantCommand(() -> ClimbUtil.setExtenderOutput(-.2))
    ));

    manualClimb.get(Button.A).whenPressed(
      new PositionArm(170)
    );

    manualClimb.get(DPad.UNPRESSED).whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> ClimbUtil.setExtenderOutput(0)),
      new InstantCommand(() -> ClimbUtil.setRotatorOutput(0)),
      new InstantCommand(() -> ClimbUtil.disableExtender()),
      new InstantCommand(() -> ClimbUtil.disableRotator())
    ));
  }

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

    // SmartDashboard.putNumber("set angle", 90);
    // controller.getButtons().X().whenPressed(new SequentialCommandGroup(
    // new InstantCommand(() -> ClimberMethods.enableRotator()), 
    // new InstantCommand(() -> ClimberMethods.setAngle(SmartDashboard.getNumber("set angle", 90))) 
    // ));

}
