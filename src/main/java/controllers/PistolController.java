package controllers;

import controllers.constants.PistolConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  }

  public double TriggerAxis(){
    return getController().getRawAxis(PistolConstants.JoystickAxis.kTriggerAxis);
  }
  public double WheelAxis(){
    return getController().getRawAxis(PistolConstants.JoystickAxis.kWheelAxis);
  }

  public Button getButtons() {
    return button;
  }
}
