package frc.robot.commands;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.cargo.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShootTest {
  CommandScheduler scheduler = null;
  Limelight limelight;
  Shooter shooter;
  Arm arm;
  Belt belt;
  Drivetrain drive;
  BallDetection ballDetection;


  // @Before
  // public void setup() {
  //   scheduler = CommandScheduler.getInstance();
  //   limelight = mock(Limelight.class);
  //   shooter = mock(Shooter.class);
  //   arm = mock(Arm.class);
  //   belt = mock(Belt.class);
  //   drive = mock(Drivetrain.class);
  //   ballDetection = mock(BallDetection.class);
  // }

  // @Test
  // public void manualShootTest() throws InterruptedException {
  //   Shoot shootCommand = new Shoot(false, false, true, 108, -2900, shooter, arm, belt, limelight, drive, ballDetection);
  //   scheduler.schedule(shootCommand);
  //   for (int i=0; i<100; i++) {
  //     scheduler.run();
  //     Thread.sleep(20);
  //   }

  //   verify(shooter).setSpeed(any());
  // }
}
