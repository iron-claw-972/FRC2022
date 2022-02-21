package frc.robot.controls;


import javax.swing.RootPaneContainer;

import controllers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.climbExtender.*;
import frc.robot.robotConstants.climbRotator.*;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoShooterWheelConstants;
import frc.robot.util.ClimberMethods;
import frc.robot.util.ShooterMethods;

public class Operator{

  public static GameController controller = new GameController(new Joystick(JoyConstants.kOperatorJoy));

  // these two are named a little weirdly because the command group for this needs to be at least a little readable
  public static TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();
  public static TraversoClimbRotatorConstants rotate = new TraversoClimbRotatorConstants();

  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoShooterWheelConstants wheelConstants = new TraversoShooterWheelConstants();

  //operator buttons
  public static void configureButtonBindings() {
    climbBinds();
    shootBinds();
  }

  public static void climbBinds() {
    controller.getDPad().up().whenPressed(new SequentialCommandGroup( new ParallelCommandGroup(
      // arm rotates to 90 degrees
      new FunctionalCommand(
        ClimberMethods::enableRotator, // on init, do this
        () -> ClimberMethods.setAngle(rotate.kNinetyDeg), // on execute, do this
        interrupted -> ClimberMethods.disableRotator(), // on end, do this
        () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
        RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
      ),

      // stow the arm
      new FunctionalCommand(
        ShooterMethods::enableWheel, 
        () -> ShooterMethods.setAngle(cargoConstants.kStowPos), 
        interrupted -> ShooterMethods.disableWheel(), 
        () -> ShooterMethods.isWheelAtSetpoint(), 
        RobotContainer.m_cargoArm
      ),

      // disable the wheel && belt
      new InstantCommand(() -> ShooterMethods.disableWheel()),
      new InstantCommand(() -> ShooterMethods.disableBelt()),

      // extender goes all the way up
      new FunctionalCommand(
        // on initialization of the command, do:
        ClimberMethods::enableExtender, 
        // when executed, do:
        () -> ClimberMethods.setExtension(extend.kMaxUpwards),
        // when the command is ended, do:
        interrupted -> ClimberMethods.disableExtender(), 
        // when this is true, end
        () -> ClimberMethods.isExtenderAtSetpoint(), 
        // object requirements
        RobotContainer.m_extenderL, RobotContainer.m_extenderR
      )
    )));

    // this extends the arm to its lowest point and extends the arm upwards a little
    controller.getDPad().down().whenPressed(new SequentialCommandGroup(    
      // extender goes to its lowest position
      new FunctionalCommand(
        ClimberMethods::enableExtender, // on init, do this
        () -> ClimberMethods.setExtension(extend.kMaxDownwards), // on execute, do this
        interrupted -> ClimberMethods.disableExtender(), // on end, do this
        () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
        RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
      ),

      // static hook should be hooked on now

      // extender goes up a little
      new FunctionalCommand(
        ClimberMethods::enableExtender, // on init, do this
        () -> ClimberMethods.setExtension(extend.kSlightlyUpward), // on execute, do this
        interrupted -> ClimberMethods.disableExtender(), // on end, do this
        () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
        RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
      )
    ));

    // the arm rotates backwards, extends, angles to the bar, compresses, && returns to 90 degrees
    controller.getDPad().right().whenPressed(new SequentialCommandGroup(

      // arm rotates max backwards
      new FunctionalCommand(
        ClimberMethods::enableRotator, // on init, do this
        () -> ClimberMethods.setAngle(rotate.kMaxBackward), // on execute, do this
        interrupted -> ClimberMethods.disableRotator(), // on end, do this
        () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
        RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
      ),

      // extender goes to its maximum point
      new FunctionalCommand(
        ClimberMethods::enableExtender, // on init, do this
        () -> ClimberMethods.setExtension(extend.kMaxUpwards), // on execute, do this
        interrupted -> ClimberMethods.disableExtender(), // on end, do this
        () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
        RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
      ),

      // rotate the arm to the bar
      new FunctionalCommand(
        ClimberMethods::enableRotator, // on init, do this
        () -> ClimberMethods.setAngle(rotate.kToBar), // on execute, do this
        interrupted -> ClimberMethods.disableRotator(), // on end, do this
        () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
        RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
      ),

      // run two commands at once
      new ParallelCommandGroup(
        // extender goes max downwards
        new FunctionalCommand(
          ClimberMethods::enableExtender, // on init, do this
          () -> ClimberMethods.setExtension(extend.kMaxDownwards), // on execute, do this
          interrupted -> ClimberMethods.disableExtender(), // on end, do this
          () -> ClimberMethods.isExtenderAtSetpoint(), // end command when this is true
          RobotContainer.m_extenderL, RobotContainer.m_extenderR // object requirements
        ),
        // arm rotates to 90 degrees
        new FunctionalCommand(
          ClimberMethods::enableRotator, // on init, do this
          () -> ClimberMethods.setAngle(rotate.kNinetyDeg), // on execute, do this
          interrupted -> ClimberMethods.disableRotator(), // on end, do this
          () -> ClimberMethods.isRotatorAtSetpoint(), // end command when this is true
          RobotContainer.m_rotatorL, RobotContainer.m_rotatorR // object requirements
        )
      )
    ));

    // resume the sequence
    controller.getButtons().START().whenPressed((new ParallelCommandGroup(
      new InstantCommand(() -> ClimberMethods.enableExtender()),
      new InstantCommand(() -> ClimberMethods.enableRotator())
    )));

    // end the sequence
    controller.getButtons().BACK().whenPressed((new ParallelCommandGroup(
      new InstantCommand(() -> ClimberMethods.disableExtender()),
      new InstantCommand(() -> ClimberMethods.disableRotator())
    )));
  }

  public static void shootBinds() {
    // get to the shooting position and shoot the ball
    controller.getButtons().RB().whenPressed(new SequentialCommandGroup(
      // move the arm to the shooting position
      new FunctionalCommand(
        ShooterMethods::enableArm, 
        () -> ShooterMethods.setAngle(cargoConstants.kFrontOutakePos), 
        interrupted -> ShooterMethods.disableArm(), 
        () -> ShooterMethods.isArmAtSetpoint(), 
        RobotContainer.m_cargoArm
      ),
      // spin the wheel and afterwards spin the belt
      new FunctionalCommand(
        ShooterMethods::enableWheel, 
        () -> ShooterMethods.setWheelSpeed(wheelConstants.kFrontOuttakeSpeed), 
        interrupted -> ShooterMethods.setBeltSpeed(beltConstants.kOuttakeSpeed), 
        () -> ShooterMethods.isWheelAtSetpoint(),
        RobotContainer.m_shooterWheel, RobotContainer.m_shooterBelt
      ),
      // keep outtaking until the ball is gone
      new WaitUntilCommand(ShooterMethods::isBallShot),
      // when the ball is gone, disable all shooter subsystems
      new InstantCommand(() -> ShooterMethods.disableAll())
    ));

    controller.getButtons().A().whenPressed(
      // move cargo arm to the back
      new FunctionalCommand(
        ShooterMethods::enableArm, 
        () -> ShooterMethods.setAngle(cargoConstants.kBackOutakePos), 
        interrupted -> ShooterMethods.disableArm(), 
        () -> ShooterMethods.isArmAtSetpoint(), 
        RobotContainer.m_cargoArm
      )
    );

    controller.getButtons().B().whenPressed(
      // move cargo arm to the front
      new FunctionalCommand(
        ShooterMethods::enableArm, 
        () -> ShooterMethods.setAngle(cargoConstants.kFrontOutakePos),
        interrupted -> ShooterMethods.disableArm(), 
        () -> ShooterMethods.isArmAtSetpoint(), 
        RobotContainer.m_cargoArm
      )
    );

    // start intaking
    controller.getButtons().X().whenPressed(new SequentialCommandGroup(
      // enable both subsystems
      new InstantCommand(() -> ShooterMethods.enableBelt()),
      new InstantCommand(() -> ShooterMethods.enableWheel()),
      // set the wheel speed
      new InstantCommand(() -> ShooterMethods.setWheelSpeed(wheelConstants.kIntakeSpeed)),
      // set the belt speed
      new InstantCommand(() -> ShooterMethods.setBeltSpeed(beltConstants.kIntakeSpeed))
    ));

    // stop intaking (after x is released)
    controller.getButtons().X().whenReleased(new SequentialCommandGroup(
      // disable the belt
      new InstantCommand(() -> ShooterMethods.disableBelt()),
      // disable the wheel
      new InstantCommand(() -> ShooterMethods.disableWheel())
    ));

    controller.getButtons().Y().whenPressed(new SequentialCommandGroup(
      // stow the arm
      new FunctionalCommand(
        ShooterMethods::enableWheel, 
        () -> ShooterMethods.setAngle(cargoConstants.kStowPos), 
        interrupted -> ShooterMethods.disableWheel(), 
        () -> ShooterMethods.isWheelAtSetpoint(), 
        RobotContainer.m_cargoArm
      )
    ));
  }
}
