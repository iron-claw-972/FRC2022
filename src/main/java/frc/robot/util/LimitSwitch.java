package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
  DigitalInput m_limitSwitch;
  Debouncer m_debouncer;
  boolean m_lastRead;

  public LimitSwitch(int port, double debouncerMargin) {
    this(new DigitalInput(port), debouncerMargin);
  }

  public LimitSwitch(int port) {
    this(port, 0);
  }

  public LimitSwitch(DigitalInput limitSwitch) {
    this(limitSwitch, 0);
  }

  public LimitSwitch(DigitalInput limitSwitch, double debouncerMargin){
    m_limitSwitch = limitSwitch;
    m_debouncer = new Debouncer(debouncerMargin, Debouncer.DebounceType.kBoth);
  }

  public boolean get() {
    //updates debounce and last read
    m_lastRead = !m_debouncer.calculate(m_limitSwitch.get());
    return !m_debouncer.calculate(m_limitSwitch.get());
  }

  public boolean fallingEdge() {
    return m_lastRead && !get();
  }
  
  public boolean risingEdge() {
    return !m_lastRead && get();
  }

  public boolean getNoUpdate(){
    return !m_debouncer.calculate(m_limitSwitch.get());
  }

}
