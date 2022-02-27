package frc.robot.controls;


import java.time.Instant;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.AlignToUpperHub;
import frc.robot.commands.GetDistance;
import frc.robot.commands.Intake;
import frc.robot.commands.PositionArm;
import frc.robot.commands.Shoot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
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
  // public static TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();
  // public static TraversoClimbRotatorConstants rotate = new TraversoClimbRotatorConstants();

  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();

  //operator buttons
  public static void configureButtonBindings() {
    // climbBinds();
    shootBinds();
    //controller.getButtons().LB().whenPressed(new InstantCommand(() -> ShooterMethods.enableAll()));

    //controller.getButtons().A().whileHeld(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));
    // controller.getButtons().X().whenPressed(new InstantCommand(() -> RobotContainer.m_cargoRotator.setPosition(0)));
    // controller.getButtons().Y().whenPressed(new InstantCommand(() -> RobotContainer.m_cargoRotator.setPosition(90)));
    

    // SmartDashboard.putNumber("Voltage output", 0);
    // controller.getButtons().Y().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoShooter.setVoltage(SmartDashboard.getNumber("Voltage output", 0))));
    // controller.getButtons().Y().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoShooter.setVoltage(0)));

    // SmartDashboard.putNumber("set speed", 0);
    // controller.getButtons().X().whenPressed(new InstantCommand(() -> RobotContainer.m_cargoShooter.setSpeed(SmartDashboard.getNumber("set speed", 0))));

    // controller.getButtons().B().whenPressed(new InstantCommand(()-> RobotContainer.m_cargoShooter.disable()));
    // controller.getButtons().A().whenPressed(new InstantCommand(()-> RobotContainer.m_cargoShooter.enable()));

    // SmartDashboard.putNumber("belt speed", 0);
    // controller.getButtons().RB().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(SmartDashboard.getNumber("belt speed", 0))));
    // controller.getButtons().RB().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(0)));

  }

  public static void climbBinds() {
    // controller.getDPad().up().whenPressed(new SequentialCommandGroup( new ParallelCommandGroup(
    //   // arm rotates to 90 degrees
    //   new FunctionalCommand(
    //     ClimberMethods::enableRotator, // on init, do this
    //     () -> ClimberMethods.setAngle(rotate.kNinetyDeg), // on execute, do this
    //     interrupted -> ClimberMethods.disableRotator(), // on end, do this
    //     () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
    //     RobotContainer.m_climbRotatorL, RobotContainer.m_climbRotatorR // object requirements
    //   ),

    //   // stow the arm
    //   new FunctionalCommand(
    //     ShooterMethods::enableWheel, 
    //     () -> ShooterMethods.setAngle(cargoConstants.kStowPos), 
    //     interrupted -> ShooterMethods.disableWheel(), 
    //     () -> ShooterMethods.isWheelAtSetpoint(), 
    //     RobotContainer.m_cargoRotator
    //   ),

    //   // disable the wheel && belt
    //   new InstantCommand(() -> ShooterMethods.disableWheel()),
    //   new InstantCommand(() -> ShooterMethods.disableBelt()),

    //   // extender goes all the way up
    //   new FunctionalCommand(
    //     // on initialization of the command, do:
    //     ClimberMethods::enableExtender, 
    //     // when executed, do:
    //     () -> ClimberMethods.setExtension(extend.kMaxUpwards),
    //     // when the command is ended, do:
    //     interrupted -> ClimberMethods.disableExtender(), 
    //     // when this is true, end
    //     () -> ClimberMethods.isExtenderAtSetpoint(), 
    //     // object requirements
    //     RobotContainer.m_extenderL, RobotContainer.m_extenderR
    //   )
    // )));

    // // this extends the arm to its lowest point and extends the arm upwards a little
    // controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
    //   // extender goes to its lowest position
    //   new FunctionalCommand(
    //     ClimberMethods::enableExtender, // on init, do this
    //     () -> ClimberMethods.setExtension(extend.kMaxDownwards), // on execute, do this
    //     interrupted -> ClimberMethods.disableExtender(), // on end, do this
    //     () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
    //     RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
    //   ),

    //   // static hook should be hooked on now

    //   // extender goes up a little
    //   new FunctionalCommand(
    //     ClimberMethods::enableExtender, // on init, do this
    //     () -> ClimberMethods.setExtension(extend.kSlightlyUpward), // on execute, do this
    //     interrupted -> ClimberMethods.disableExtender(), // on end, do this
    //     () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
    //     RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
    //   )
    // ));

    // // the arm rotates backwards, extends, angles to the bar, compresses, && returns to 90 degrees
    // controller.getDPad().right().whenPressed(new SequentialCommandGroup(

    //   // arm rotates max backwards
    //   new FunctionalCommand(
    //     ClimberMethods::enableRotator, // on init, do this
    //     () -> ClimberMethods.setAngle(rotate.kMaxBackward), // on execute, do this
    //     interrupted -> ClimberMethods.disableRotator(), // on end, do this
    //     () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
    //     RobotContainer.m_climbRotatorL, RobotContainer.m_climbRotatorR // object requirements
    //   ),

    //   // extender goes to its maximum point
    //   new FunctionalCommand(
    //     ClimberMethods::enableExtender, // on init, do this
    //     () -> ClimberMethods.setExtension(extend.kMaxUpwards), // on execute, do this
    //     interrupted -> ClimberMethods.disableExtender(), // on end, do this
    //     () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
    //     RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
    //   ),

    //   // rotate the arm to the bar
    //   new FunctionalCommand(
    //     ClimberMethods::enableRotator, // on init, do this
    //     () -> ClimberMethods.setAngle(rotate.kToBar), // on execute, do this
    //     interrupted -> ClimberMethods.disableRotator(), // on end, do this
    //     () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
    //     RobotContainer.m_climbRotatorL, RobotContainer.m_climbRotatorR // object requirements
    //   ),

    //   // run two commands at once
    //   new ParallelCommandGroup(
    //     // extender goes max downwards
    //     new FunctionalCommand(
    //       ClimberMethods::enableExtender, // on init, do this
    //       () -> ClimberMethods.setExtension(extend.kMaxDownwards), // on execute, do this
    //       interrupted -> ClimberMethods.disableExtender(), // on end, do this
    //       () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
    //       RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
    //     ),
    //     // arm rotates to 90 degrees
    //     new FunctionalCommand(
    //       ClimberMethods::enableRotator, // on init, do this
    //       () -> ClimberMethods.setAngle(rotate.kNinetyDeg), // on execute, do this
    //       interrupted -> ClimberMethods.disableRotator(), // on end, do this
    //       () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
    //       RobotContainer.m_climbRotatorL, RobotContainer.m_climbRotatorR // object requirements
    //     )
    //   )
    // ));

    // // resume the sequence
    // controller.getButtons().START().whenPressed((new ParallelCommandGroup(
    //   new InstantCommand(() -> ClimberMethods.enableExtender()),
    //   new InstantCommand(() -> ClimberMethods.enableRotator())
    // )));

    // // end the sequence
    // controller.getButtons().BACK().whenPressed((new ParallelCommandGroup(
    //   new InstantCommand(() -> ClimberMethods.disableExtender()),
    //   new InstantCommand(() -> ClimberMethods.disableRotator())
    // )));
  }

  public static void shootBinds() {
    controller.getButtons().RB().whenPressed(new ConditionalCommand(
      // new Shoot(cargoConstants.kFrontOuttakeFarPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeFarSpeed, beltConstants.kOuttakeSpeed, true),
      // new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, wheelConstants.kBackOuttakeFarSpeed, beltConstants.kOuttakeSpeed, true),
      new Shoot(cargoConstants.kFrontOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true),
      new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true),
      ShooterMethods::isArmFront
    ));
    controller.getButtons().RT().whenActive(new ConditionalCommand(
      new Shoot(cargoConstants.kFrontOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kFrontOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false),
      new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, wheelConstants.kBackOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false),
      ShooterMethods::isArmFront
    ));

    controller.getButtons().A().whenPressed(new PositionArm(cargoConstants.kBackOuttakeFarPos));
    controller.getButtons().B().whenPressed(new PositionArm(cargoConstants.kFrontOuttakeFarPos));
    controller.getButtons().X().whenPressed(new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kFrontOuttakeFarPos, true, Constants.kIsRedAlliance)); 
    // controller.getButtons().Y().whenPressed(new PositionArm(cargoConstants.kStowPos));
    controller.getButtons().Y().whenPressed(new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive));
    // controller.getButtons().RB().whenPressed(new GetDistance(RobotContainer.m_limelight, RobotContainer.m_cargoRotator));
  }

  // public static void cargoTestBinds() {
  //   // controller.getButtons().RB().whenPressed(new SequentialCommandGroup(
  //   // ));

  //   controller.getButtons().LB().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(controller.getJoystickAxis().leftY())));
  //   controller.getButtons().LB().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(0)));
    
  //   controller.getButtons().RB().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(-controller.getJoystickAxis().rightY())));
  //   controller.getButtons().RB().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(0)));


  //   SmartDashboard.putNumber("Shooter", 0);
  //   controller.getButtons().X().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(SmartDashboard.getNumber("Shooter X", 0))));
  //   controller.getButtons().X().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(0)));

  //   SmartDashboard.putNumber("belt", 0);
  //   controller.getButtons().B().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(-SmartDashboard.getNumber("belt B", 0))));
  //   controller.getButtons().B().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(0)));

  //   SmartDashboard.putNumber("Shooter", 0);
  //   controller.getButtons().Y().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(SmartDashboard.getNumber("Shooter Y", 0))));
  //   controller.getButtons().Y().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoShooter.setOutput(0)));

  //   SmartDashboard.putNumber("belt", 0);
  //   controller.getButtons().A().whileHeld(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(-SmartDashboard.getNumber("belt A", 0))));
  //   controller.getButtons().A().whenReleased(new InstantCommand(() -> RobotContainer.m_cargoBelt.setOutput(0)));
  // }
}