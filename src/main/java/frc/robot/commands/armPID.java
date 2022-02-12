package frc.robot.commands;

import frc.robot.subsystems.TestArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.Operator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class armPID extends CommandBase {
  private final TestArm m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public armPID(TestArm subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // m_subsystem.setEncoderRad(0.0);
    // m_subsystem.setGoalRad(1.5);
    // m_subsystem.m_motor.set(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_subsystem.posPID();
    // System.out.println(m_subsystem.getPosition());
    //if (Operator.controller.getButtons().X().getAsBoolean()){
    //   m_subsystem.setRaw(Operator.controller.getJoystickAxis().leftY()*0.05);
    //}
    SmartDashboard.putNumber("Bore Encoder", m_subsystem.currentAngle());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}