package controllers;

import controllers.constants.kMadCatzC;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class MadCatzC extends Controller{

    public MadCatzC(Joystick joystick_){
        super(joystick_);
    }

    //returns JoystickButton object
    Button Button = new Button();
    public class Button {
        public JoystickButton b1() {
            return new JoystickButton(getController(), kMadCatzC.buttons.k1);
        }
        public JoystickButton b2() {
            return new JoystickButton(getController(), kMadCatzC.buttons.k2);
        }
        public JoystickButton b3() {
            return new JoystickButton(getController(), kMadCatzC.buttons.k3);
        }
        public JoystickButton b4() {
            return new JoystickButton(getController(), kMadCatzC.buttons.k4);
        }
        public JoystickButton b5() {
            return new JoystickButton(getController(), kMadCatzC.buttons.k5);
        }
        public JoystickButton b6() {
            return new JoystickButton(getController(), kMadCatzC.buttons.k6);
        }
        public JoystickButton b7() {
            return new JoystickButton(getController(), kMadCatzC.buttons.k7);
        }
    }

    //returns POVButton object
    public HatSwitch HatSwitch = new HatSwitch();
    public class HatSwitch {
        public POVButton up() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kUp);
        }
        public POVButton upRight() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kUpRight);
        }
        public POVButton right() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kRight);
        }
        public POVButton downRight() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kDownRight);
        }
        public POVButton down() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kDown);
        }
        public POVButton downLeft() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kDownLeft);
        }
        public POVButton left() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kLeft);
        }
        public POVButton upLeft() {
            return new POVButton(getController(), kMadCatzC.thumbstick.kUpLeft);
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
    public JoystickAxis JoystickAxis = new JoystickAxis();
    public class JoystickAxis{
        public double X(){
            return getController().getRawAxis(kMadCatzC.joystickAxis.kX);
        }
        public double Y(){
            return getController().getRawAxis(kMadCatzC.joystickAxis.kY);
        }
        public double zRotate(){
            return getController().getRawAxis(kMadCatzC.joystickAxis.kZRotate);
        }
        public double zAxis(){
            return getController().getRawAxis(kMadCatzC.joystickAxis.kZAxis);
        }
    }
}
