package frc.robot.commands;

import controllers.GameController;
import controllers.GameController.RumbleStatus;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Rumble extends SequentialCommandGroup{

  private GameController m_controller;

  public Rumble(GameController controller) {
    this.m_controller = controller;
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() -> controller.setRumble(RumbleStatus.RUMBLE_ON)),
        new WaitCommand(.5),
        new InstantCommand(() -> controller.setRumble(RumbleStatus.RUMBLE_OFF))
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(RumbleStatus.RUMBLE_OFF);
  }
}
