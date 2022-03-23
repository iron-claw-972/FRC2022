package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
    
  DigitalInput limitSwitch;
  Debouncer debouncer;
  boolean lastRead;

  public LimitSwitch(int port){
    this(port, 0);
  }

  public LimitSwitch(int port, double debouncerMargin){
    limitSwitch = new DigitalInput(port);
    debouncer = new Debouncer(debouncerMargin, Debouncer.DebounceType.kBoth);
  }

  public boolean get() {
    //updates debounce and last read
    lastRead = !debouncer.calculate(limitSwitch.get());
    return !debouncer.calculate(limitSwitch.get());
  }

  public boolean fallingEdge() {
    return lastRead && !get();
  }
  
  public boolean risingEdge() {
    return !lastRead && get();
  }

  public boolean getNoUpdate(){
    return !debouncer.calculate(limitSwitch.get());
  }

}
