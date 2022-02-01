package controllers;

import controllers.constants.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class GameC extends Controller{

    public GameC(Joystick joystick_){
        super(joystick_);
    }

    //returns JoystickButton object
    public Button Button = new Button();
    public final class Button {
        public JoystickButton A() {
            return new JoystickButton(getController(), kGameC.buttons.kA);
        }
        public JoystickButton B() {
            return new JoystickButton(getController(), kGameC.buttons.kB);
        }
        public JoystickButton X() {
            return new JoystickButton(getController(), kGameC.buttons.kX);
        }
        public JoystickButton Y() {
            return new JoystickButton(getController(), kGameC.buttons.kY);
        }
        public JoystickButton LB() {
            return new JoystickButton(getController(), kGameC.buttons.kLB);
        }
        public JoystickButton RB() {
            return new JoystickButton(getController(), kGameC.buttons.kRB);
        }
        public JoystickButton back() {
            return new JoystickButton(getController(), kGameC.buttons.kBack);
        }
        public JoystickButton start() {
            return new JoystickButton(getController(), kGameC.buttons.kStart);
        }
    }
    
    //returns POVButton object
    public DPad DPad = new DPad();
    public class DPad {

        public POVButton unpressed(){
            return  new POVButton(getController(), -1);
        }

        public POVButton up() {
            return new POVButton(getController(), kGameC.dPad.kUp);
        }
        public POVButton upRight() {
            return new POVButton(getController(), kGameC.dPad.kUpRight);
        }
        public POVButton right() {
            return new POVButton(getController(), kGameC.dPad.kRight);
        }
        public POVButton downRight() {
            return new POVButton(getController(), kGameC.dPad.kDownRight);
        }
        public POVButton down() {
            return new POVButton(getController(), kGameC.dPad.kDown);
        }
        public POVButton downLeft() {
            return new POVButton(getController(), kGameC.dPad.kDownLeft);
        }
        public POVButton left() {
            return new POVButton(getController(), kGameC.dPad.kLeft);
        }
        public POVButton upLeft() {
            return new POVButton(getController(), kGameC.dPad.kUpLeft);
        }
        
        public Trigger allUp(){
            return up()
            .or(upRight()
            .or(upLeft()));
        }
        public Trigger allDown(){
            return down()
            .or(downRight()
            .or(downLeft()));
        }
        public Trigger allLeft(){
            return left()
            .or(upLeft()
            .or(downLeft()));
        }
        public Trigger allRight(){
            return right()
            .or(upRight()
            .or(downRight()));
        }
        
    }

    //returns Joystick Axis value
    public JoystickAxis JoystickAxis = new JoystickAxis();
    public class JoystickAxis{
        public double leftX(){
            return getController().getRawAxis(kGameC.joystickAxis.kLeftX);
        }
        public double leftY(){
            return getController().getRawAxis(kGameC.joystickAxis.kLeftY);
        }
        public double rightX(){
            return getController().getRawAxis(kGameC.joystickAxis.kRightX);
        }
        public double rightY(){
            return getController().getRawAxis(kGameC.joystickAxis.kRightY);
        }
    }

    //returns Trigger Axis value
    public TriggerAxis TriggerAxis = new TriggerAxis();
    public  class TriggerAxis{
        public double leftTrigger(){
            return getController().getRawAxis(kGameC.triggers.kLeftT);
        }
        public double RightTrigger(){
            return getController().getRawAxis(kGameC.triggers.kRightT);
        }
    }
    
    
}
