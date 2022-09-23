package frc.robot.controls;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.cargo.EjectBall;
import frc.robot.commands.cargo.Intake;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.util.Functions;
import lib.controllers.*;
import lib.controllers.GameController.GCAxis;
import lib.controllers.PistolController.PistolAxis;
import lib.controllers.PistolController.Button;

public class Driver {
  private static PistolController driverP = new PistolController(Constants.oi.kDriverJoyPistol);
  private static GameController driverGC = new GameController(Constants.oi.kDriverJoyGC); 

  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(Constants.drive.kSlewRate);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(Constants.drive.kSlewRate);

  // driver buttons
  public static void configureControls() {
    SmartDashboard.setDefaultBoolean("Pistol Driver", true);
    if (!DriverStation.isJoystickConnected(Constants.oi.kDriverJoyPistol)) {
      // Don't try to configure bindings if controller not plugged in
      DriverStation.reportWarning("Driver controller not connected to Port " + Constants.oi.kDriverJoyPistol, true);
      return;
    }
    if (getThrottleValue() != 0 || getTurnValue() != 0) {
      // Make sure driver controller is responding so the drivetrain doesn't go crazy upon enabling
      DriverStation.reportWarning("Driver controller not properly connected to Port " + Constants.oi.kDriverJoyPistol + ". Try restarting or reconnecting driver controller.", false);
      return;
    }
    configureDriveControls();
  }

  private static void configureDriveControls() {
    // Position arm front
    driverP.get(Button.BOTTOM_FRONT).whenPressed(new PositionArm(Constants.arm.kFrontLimelightScanPos));

    // Position arm back
    driverP.get(Button.BOTTOM_BACK).whenPressed(new PositionArm(Constants.arm.kBackLimelightScanPos));

    // Intake w/ ball chase for red ball
    driverP.get(Button.TOP_FRONT)
      .whenHeld(new Intake(Constants.arm.kUprightPos, true, true))
      .whenReleased(new PositionArm(Constants.arm.kUprightPos).andThen(() -> Robot.ll.setUpperHubPipeline()));

    // Intake w/ ball chase for blue ball
    driverP.get(Button.TOP_BACK)
      .whenHeld(new Intake(Constants.arm.kUprightPos, true, false))
      .whenReleased(new PositionArm(Constants.arm.kUprightPos).andThen(() -> Robot.ll.setUpperHubPipeline()));

    // Eject ball
    driverP.get(Button.BOTTOM).whenHeld(new EjectBall());
  }
  
  public static double getThrottleValue() {
    // put any processes in any order of the driver's choosing
    // Controllers y-axes are natively up-negative, down-positive
      return -slewThrottle.calculate(Functions.deadband(Constants.oi.kDeadband, getRawThrottleValue()));
  }

  public static double getTurnValue() {
    // right is positive; left is negative
    return -slewTurn.calculate(Functions.deadband(Constants.oi.kDeadband, getRawTurnValue()));
  }

  public static double getRawThrottleValue() {
    // Controllers y-axes are natively up-negative, down-positive
    if (SmartDashboard.getBoolean("Pistol Driver", true)) {
      return driverP.get(PistolAxis.TRIGGER);
    }
    return driverGC.get(GCAxis.LEFT_Y);
  }

  public static double getRawTurnValue() {
    // Right is Positive left is negative
    if (SmartDashboard.getBoolean("Pistol Driver", true)) {
      return driverP.get(PistolAxis.WHEEL);
    }
    return driverGC.get(GCAxis.RIGHT_X);
  }
}
