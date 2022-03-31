package frc.robot.controls;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.commands.cargo.AlignToUpperHub;
import frc.robot.commands.cargo.EjectBall;
import frc.robot.commands.cargo.Intake;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.util.Functions;
import lib.controllers.*;
import lib.controllers.PistolController.Axis;
import lib.controllers.PistolController.Button;

public class Driver {
  private static PistolController driver = new PistolController(Constants.oi.kDriverJoy);

  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(Constants.drive.kSlewRate);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(Constants.drive.kSlewRate);

  // driver buttons
  public static void configureControls() {
    if (!DriverStation.isJoystickConnected(Constants.oi.kDriverJoy)) {
      // Don't try to configure bindings if controller not plugged in
      DriverStation.reportWarning("Driver controller not connected to Port " + Constants.oi.kDriverJoy, true);
      return;
    }
    if (getThrottleValue() != 0 || getTurnValue() != 0) {
      // Make sure driver controller is responding so the drivetrain doesn't go crazy upon enabling
      DriverStation.reportWarning("Driver controller not properly connected to Port " + Constants.oi.kDriverJoy + ". Try restarting or reconnecting driver controller.", false);
      return;
    }
    configureDriveControls();
  }

  private static void configureDriveControls() {
    // Position arm front
    driver.get(Button.BOTTOM_FRONT).whenPressed(new PositionArm(Constants.arm.kFrontLimelightScanPos));

    // Position arm back
    driver.get(Button.BOTTOM_BACK).whenPressed(new PositionArm(Constants.arm.kBackLimelightScanPos));

    // Intake w/ ball chase for red ball
    driver.get(Button.TOP_FRONT)
      .whenHeld(new Intake(Constants.arm.kOptimalBackShootingPos, true, true))
      .whenReleased(new PositionArm(Constants.arm.kUprightPos).andThen(() -> Robot.ll.setUpperHubPipeline()));

    // Intake w/ ball chase for blue ball
    driver.get(Button.TOP_BACK)
      .whenHeld(new Intake(Constants.arm.kOptimalBackShootingPos, true, false))
      .whenReleased(new PositionArm(Constants.arm.kUprightPos).andThen(() -> Robot.ll.setUpperHubPipeline()));

    // Eject ball
    driver.get(Button.BOTTOM).whenHeld(new EjectBall());
    // driver.get(Button.BOTTOM).whenHeld(new AlignToUpperHub());

  }
  
  public static double getThrottleValue() {
    // put any processes in any order of the driver's choosing
    // Controllers y-axes are natively up-negative, down-positive
    return slewThrottle.calculate(Functions.deadband(Constants.oi.kDeadband, getRawThrottleValue()));
  }

  public static double getTurnValue() {
    // right is positive; left is negative
    return slewTurn.calculate(Functions.deadband(Constants.oi.kDeadband, getRawTurnValue()));
  }

  public static double getRawThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive
    return driver.get(Axis.TRIGGER);
  }

  public static double getRawTurnValue() {
    // Right is Positive left is negative
    return driver.get(Axis.WHEEL);
  }
}
