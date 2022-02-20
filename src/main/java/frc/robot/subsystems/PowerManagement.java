package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerManagement extends SubsystemBase {

  private static PowerManagement instance;

  private final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  public static PowerManagement getInstance() {
    if (instance == null) {
      instance = new PowerManagement();
    }
    return instance;
  }

  public void printPowerUsage() {
    double voltage = pdp.getVoltage();
    //double totalCurrent = pdp.getTotalCurrent();
    double totalPower = pdp.getTotalPower();
    double totalEnergy = pdp.getTotalEnergy();
    System.out.println("Voltage: " + voltage);
    //System.out.println("Total current: " + totalCurrent);
    System.out.println("Total power: " + totalPower);
    System.out.println("Total energy: " + totalEnergy);

    double leftMotor1Current = pdp.getCurrent(14);
    double leftMotor2Current = pdp.getCurrent(15);
    double rightMotor1Current = pdp.getCurrent(0);
    double rightMotor2Current = pdp.getCurrent(1);

    System.out.println("Left Motor 1 Current: " + leftMotor1Current);
    System.out.println("Left Motor 2 Current: " + leftMotor2Current);
    System.out.println("Right Motor 1 Current: " + rightMotor1Current);
    System.out.println("Right Motor 2 Current: " + rightMotor2Current);

    System.out.println("\n");
  }

}
