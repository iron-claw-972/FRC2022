package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PistolController extends Controller {
  public final Trigger
    TOP_BACK_ONLY = get(PistolButton.TOP_BACK).and(get(PistolButton.TOP_FRONT).negate()),
    TOP_FRONT_ONLY = get(PistolButton.TOP_FRONT).and(get(PistolButton.TOP_BACK).negate()),
    BOTTOM_BACK_ONLY = get(PistolButton.BOTTOM_BACK).and(get(PistolButton.BOTTOM_FRONT).negate()),
    BOTTOM_FRONT_ONLY = get(PistolButton.BOTTOM_FRONT).and(get(PistolButton.BOTTOM_BACK).negate());


  public PistolController(int port) {
    super(port);
  }
  
  public enum PistolButton {
    TOP_BACK(1), TOP_FRONT(2),BOTTOM_FRONT(3), BOTTOM_BACK(4),  BOTTOM(5);

    public final int id;

    PistolButton(final int id) {
      this.id = id;
    }
  }
  
  public enum PistolAxis {
    WHEEL(0), TRIGGER(1);

    public final int id;

    PistolAxis(final int id) {
      this.id = id;
    }
  }

  public JoystickButton get(PistolButton button) {
    return new JoystickButton(m_controller, button.id);
  }

  public double get(PistolAxis axis) {
    return m_controller.getRawAxis(axis.id);
  }
  
  public Trigger get(Trigger trigger) {
    return trigger;
  }

  public Joystick get() {
    return m_controller;
  }
}
