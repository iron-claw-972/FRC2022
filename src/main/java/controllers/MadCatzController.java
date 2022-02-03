package controllers;

import controllers.constants.MadCatzConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class MadCatzController extends Controller{

    private Button Button = new Button();
    private HatSwitch HatSwitch = new HatSwitch();
    private JoystickAxis JoystickAxis = new JoystickAxis();

    public MadCatzController(Joystick joystick_){
        super(joystick_);
    }

    //returns JoystickButton object
    public class Button {
        public JoystickButton b1() {
            return new JoystickButton(getController(), MadCatzConstants.Buttons.k1);
        }
        public JoystickButton b2() {
            return new JoystickButton(getController(), MadCatzConstants.Buttons.k2);
        }
        public JoystickButton b3() {
            return new JoystickButton(getController(), MadCatzConstants.Buttons.k3);
        }
        public JoystickButton b4() {
            return new JoystickButton(getController(), MadCatzConstants.Buttons.k4);
        }
        public JoystickButton b5() {
            return new JoystickButton(getController(), MadCatzConstants.Buttons.k5);
        }
        public JoystickButton b6() {
            return new JoystickButton(getController(), MadCatzConstants.Buttons.k6);
        }
        public JoystickButton b7() {
            return new JoystickButton(getController(), MadCatzConstants.Buttons.k7);
        }
    }

    //returns POVButton object
    public class HatSwitch {
        public POVButton up() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kUp);
        }
        public POVButton upRight() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kUpRight);
        }
        public POVButton right() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kRight);
        }
        public POVButton downRight() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kDownRight);
        }
        public POVButton down() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kDown);
        }
        public POVButton downLeft() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kDownLeft);
        }
        public POVButton left() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kLeft);
        }
        public POVButton upLeft() {
            return new POVButton(getController(), MadCatzConstants.Thumbstick.kUpLeft);
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

    //returns JoystickButton object
    public class JoystickAxis{
        public double X(){
            return getController().getRawAxis(MadCatzConstants.JoystickAxis.kX);
        }
        public double Y(){
            return getController().getRawAxis(MadCatzConstants.JoystickAxis.kY);
        }
        public double zRotate(){
            return getController().getRawAxis(MadCatzConstants.JoystickAxis.kZRotate);
        }
        public double zAxis(){
            return getController().getRawAxis(MadCatzConstants.JoystickAxis.kZAxis);
        }
    }
}
