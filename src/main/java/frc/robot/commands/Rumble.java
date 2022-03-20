package frc.robot.commands;

import controllers.GameController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Rumble extends SequentialCommandGroup {
  public Rumble(GameController controller) {
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(() -> controller.startRumble()),
            new WaitCommand(.5),
            new InstantCommand(() -> controller.endRumble())));
  }
}