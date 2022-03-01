package frc.robot.controls;


import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.Intake;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.ClimberMove;
import frc.robot.commands.PositionArm;
import frc.robot.commands.Shoot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;
import frc.robot.util.ClimberMethods;
import frc.robot.util.ShooterMethods;

public class Operator {

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  // these two are named a little weirdly because the command group for this needs to be at least a little readable
  public static TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();
  public static TraversoClimbRotatorConstants rotate = new TraversoClimbRotatorConstants();

  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();

  //operator buttons
  public static void configureButtonBindings() {
    climbBinds();
    shootBinds();
  }

  public static void climbBinds() {
    controller.getDPad().up().whenPressed(new ParallelCommandGroup(
      // move the cargo arm to stow
      new PositionArm(cargoConstants.kStowPos),

      // extend upwards, go an angle where we can hook the static hook
      new ClimberMove(extend.kMaxUpwards, rotate.kHookStatic)
    ));

    controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
      // extend downwards, go to 90 degrees
      new ClimberMove(extend.kMaxDownwards, rotate.kNinetyDeg),

      // by now, the static hooks should be on the bar

      // extend slightly upward, remain 90 degrees
      new ClimberMove(extend.kSlightlyUpward, rotate.kNinetyDeg)
    ));

    controller.getDPad().right().whenPressed(new SequentialCommandGroup(
      // go upwards and rotate backwards
      new ClimberMove(extend.kMaxUpwards, rotate.kMaxBackward),
      // remain going upwards and rotate towards the bar
      new ClimberMove(extend.kMaxUpwards, rotate.kToBar),
      // compress and rotate to 90 degrees
      new ClimberMove(extend.kMaxDownwards, rotate.kNinetyDeg)
    ));

    // resume the sequence
    controller.getButtons().START().whenPressed(
      new InstantCommand(() -> ClimberMethods.enableAll()));

    // pause the sequence
    controller.getButtons().BACK().whenPressed(
      new InstantCommand(() -> ClimberMethods.disableAll()));
  }

  public static void shootBinds() {
    controller.getButtons().RB().whenPressed(new ConditionalCommand(
      new Shoot(cargoConstants.kFrontOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true),
      new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true),
      ShooterMethods::isArmFront
    ));

    controller.getButtons().RT().whenActive(new ConditionalCommand(
      // shoot at a desired angle and outtake/intake speeds
      new Shoot(cargoConstants.kFrontOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false),
      new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kBackOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false),
      ShooterMethods::isArmFront
    ));


    // move arm to back
    controller.getButtons().A().whenPressed(new PositionArm(cargoConstants.kBackOuttakeFarPos));
    // move arm to front
    controller.getButtons().B().whenPressed(new PositionArm(cargoConstants.kFrontOuttakeFarPos));
    controller.getButtons().X().whenPressed(new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kFrontOuttakeFarPos, false, Constants.kIsRedAlliance)); 
    // controller.getButtons().Y().whenPressed(new PositionArm(cargoConstants.kStowPos));
    controller.getButtons().Y().whenPressed(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));
    // controller.getButtons().RB().whenPressed(new GetDistance(RobotContainer.m_limelight, RobotContainer.m_cargoRotator));
  }

  public static void cargoTestBinds() {
    // controller.getButtons().RB().whenPressed(new SequentialCommandGroup(
    // ));

    controller.getButtons().LB().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(controller.getJoystickAxis().leftY())));
    controller.getButtons().LB().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(0)));
    
    controller.getButtons().RB().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(-controller.getJoystickAxis().rightY())));
    controller.getButtons().RB().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(0)));


    SmartDashboard.putNumber("Shooter", 0);
    controller.getButtons().X().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(SmartDashboard.getNumber("Shooter X", 0))));
    controller.getButtons().X().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(0)));

    SmartDashboard.putNumber("belt", 0);
    controller.getButtons().B().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(-SmartDashboard.getNumber("belt B", 0))));
    controller.getButtons().B().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(0)));

    SmartDashboard.putNumber("Shooter", 0);
    controller.getButtons().Y().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(SmartDashboard.getNumber("Shooter Y", 0))));
    controller.getButtons().Y().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(0)));

    SmartDashboard.putNumber("belt", 0);
    controller.getButtons().A().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(-SmartDashboard.getNumber("belt A", 0))));
    controller.getButtons().A().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(0)));
  }
}