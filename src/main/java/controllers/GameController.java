package controllers;

import controllers.constants.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class GameController extends Controller {
  
  private Button Button = new Button();
  private DPad DPad = new DPad();
  private TriggerAxis TriggerAxis = new TriggerAxis();
  private JoystickAxis JoystickAxis = new JoystickAxis();
  private Joystick Joystick;
  
  public GameController(Joystick joystick_) {
    super(joystick_);
    Joystick = joystick_;
  }
  
  public JoystickAxis getJoystickAxis() {
    return JoystickAxis;
  }

  public TriggerAxis getTriggerAxis() {
    return TriggerAxis;
  }

  public DPad getDPad() {
    return DPad;
  }

  public Button getButtons() {
    return Button;
  }

  public void startRumble() {
    Joystick.setRumble(GenericHID.RumbleType.kLeftRumble, .7);
    Joystick.setRumble(GenericHID.RumbleType.kRightRumble, .7);
  }

  public void endRumble() {
    Joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    Joystick.setRumble(GenericHID.RumbleType.kRightRumble, 0);
  }

  //returns JoystickButton object
  public final class Button {
    public JoystickButton A() {
      return new JoystickButton(getController(), GameConstants.buttons.kA);
    }
    public JoystickButton B() {
      return new JoystickButton(getController(), GameConstants.buttons.kB);
    }
    public JoystickButton X() {
      return new JoystickButton(getController(), GameConstants.buttons.kX);
    }
    public JoystickButton Y() {
      return new JoystickButton(getController(), GameConstants.buttons.kY);
    }
    public JoystickButton LB() {
      return new JoystickButton(getController(), GameConstants.buttons.kLB);
    }
    public JoystickButton RB() {
      return new JoystickButton(getController(), GameConstants.buttons.kRB);
    }
    public JoystickButton BACK() {
      return new JoystickButton(getController(), GameConstants.buttons.kBack);
    }
    public JoystickButton START() {
      return new JoystickButton(getController(), GameConstants.buttons.kStart);
    }
    public Trigger LT() {
      return new Trigger(TriggerAxis::leftTriggerButton);
    }
    public Trigger RT() {
      return new Trigger(TriggerAxis::rightTriggerButton);
    }
    public JoystickButton leftJoyButton() {
      return new JoystickButton(getController(), GameConstants.buttons.kLeftJoyPressed);
    }
    public JoystickButton rightJoyButton() {
      return new JoystickButton(getController(), GameConstants.buttons.kRightJoyPressed);
    }
  }
  
  //returns POVButton object
  public class DPad {
    public POVButton unpressed() {
      return new POVButton(getController(), -1);
    }
    public POVButton up() {
      return new POVButton(getController(), GameConstants.dPad.kUp);
    }
    public POVButton upRight() {
      return new POVButton(getController(), GameConstants.dPad.kUpRight);
    }
    public POVButton right() {
      return new POVButton(getController(), GameConstants.dPad.kRight);
    }
    public POVButton downRight() {
      return new POVButton(getController(), GameConstants.dPad.kDownRight);
    }
    public POVButton down() {
      return new POVButton(getController(), GameConstants.dPad.kDown);
    }
    public POVButton downLeft() {
      return new POVButton(getController(), GameConstants.dPad.kDownLeft);
    }
    public POVButton left() {
      return new POVButton(getController(), GameConstants.dPad.kLeft);
    }
    public POVButton upLeft() {
      return new POVButton(getController(), GameConstants.dPad.kUpLeft);
    }   
    public Trigger allUp() {
      return up()
      .or(upRight()
      .or(upLeft()));
    }
    public Trigger allDown() {
      return down()
      .or(downRight()
      .or(downLeft()));
    }
    public Trigger allLeft() {
      return left()
      .or(upLeft()
      .or(downLeft()));
    }
    public Trigger allRight() {
      return right()
      .or(upRight()
      .or(downRight()));
    }
  }
  
  //returns Joystick Axis value
  public class JoystickAxis{
    public double leftX() {
      return getController().getRawAxis(GameConstants.joystickAxis.kLeftX);
    }
    public double leftY() {
      return getController().getRawAxis(GameConstants.joystickAxis.kLeftY);
    }
    public double rightX() {
      return getController().getRawAxis(GameConstants.joystickAxis.kRightX);
    }
    public double rightY() {
      return getController().getRawAxis(GameConstants.joystickAxis.kRightY);
    }
  }
  
  //returns Trigger Axis value
  public class TriggerAxis {
    public double leftTrigger() {
      return getController().getRawAxis(GameConstants.triggers.kLeftT);
    }
    public boolean leftTriggerButton(){
      return leftTrigger() > 0.5;
    }
    public double rightTrigger() {
      return getController().getRawAxis(GameConstants.triggers.kRightT);
    }
    public boolean rightTriggerButton(){
      return rightTrigger() > 0.5;
    }
  }
}