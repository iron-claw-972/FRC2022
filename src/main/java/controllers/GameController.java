package controllers;

import controllers.constants.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class GameController extends Controller {
  
  private Button Button = new Button();
  private DPad DPad = new DPad();
  private TriggerAxis TriggerAxis = new TriggerAxis();
  private JoystickAxis JoystickAxis = new JoystickAxis();
  
  public GameController(Joystick joystick_) {
    super(joystick_);
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
    public JoystickButton back() {
      return new JoystickButton(getController(), GameConstants.buttons.kBack);
    }
    public JoystickButton start() {
      return new JoystickButton(getController(), GameConstants.buttons.kStart);
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
    public double rightTrigger() {
      return getController().getRawAxis(GameConstants.triggers.kRightT);
    }
  }
  
  
}