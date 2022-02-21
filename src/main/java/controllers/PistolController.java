package controllers;

import controllers.constants.PistolConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PistolController extends Controller {

  private Button button = new Button();

  public PistolController(Joystick joystick_){
    super(joystick_);
  }

  public class Button {
    public JoystickButton backSwitchTop() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kBackSwitchTop);
    }
    public JoystickButton frontSwitchTop() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kForwardSwitchTop);
    }
    public JoystickButton backSwitchBottom() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kBackSwitchBottom);
    }
    public JoystickButton frontSwitchBottom() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kForwardSwitchBottom);
    }
    public JoystickButton bottomButton() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kBottomButton);
    }

    public Trigger backSwitchTopSmart() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kBackSwitchTop).and(frontSwitchTop().negate());
    }
    public Trigger frontSwitchTopSmart() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kForwardSwitchTop).and(backSwitchTop().negate());
    }
    public Trigger backSwitchBottomSmart() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kBackSwitchBottom).and(frontSwitchBottom().negate());
    }
    public Trigger frontSwitchBottomSmart() {
      return new JoystickButton(getController(), PistolConstants.Buttons.kForwardSwitchBottom).and(frontSwitchBottom().negate());
    }
  }

  public double getTriggerAxis() {
    return getController().getRawAxis(PistolConstants.JoystickAxis.kTriggerAxis);
  }
  public double getWheelAxis() {
    return getController().getRawAxis(PistolConstants.JoystickAxis.kWheelAxis);
  }

  public Button getButtons() {
    return button;
  }
}
