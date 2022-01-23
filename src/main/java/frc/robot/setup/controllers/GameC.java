package frc.robot.setup.controllers;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.constants.controller.*;

public class GameC extends Controller{

    public GameC(Joystick joystick_){
        super(joystick_);
    }

    public Button Button = new Button();
    public final class Button {
        public JoystickButton A() {
            return new JoystickButton(controller, kGameC.buttons.kA);
        }
        public JoystickButton B() {
            return new JoystickButton(controller, kGameC.buttons.kB);
        }
        public JoystickButton X() {
            return new JoystickButton(controller, kGameC.buttons.kX);
        }
        public JoystickButton Y() {
            return new JoystickButton(controller, kGameC.buttons.kY);
        }
        public JoystickButton LB() {
            return new JoystickButton(controller, kGameC.buttons.kLB);
        }
        public JoystickButton RB() {
            return new JoystickButton(controller, kGameC.buttons.kRB);
        }
        public JoystickButton back() {
            return new JoystickButton(controller, kGameC.buttons.kBack);
        }
        public JoystickButton start() {
            return new JoystickButton(controller, kGameC.buttons.kStart);
        }
    }
    
    public DPad DPad = new DPad();
    public class DPad {
        public POVButton up() {
            return new POVButton(controller, kGameC.dPad.kUp);
        }
        public POVButton upRight() {
            return new POVButton(controller, kGameC.dPad.kUpRight);
        }
        public POVButton right() {
            return new POVButton(controller, kGameC.dPad.kRight);
        }
        public POVButton downRight() {
            return new POVButton(controller, kGameC.dPad.kDownRight);
        }
        public POVButton down() {
            return new POVButton(controller, kGameC.dPad.kDown);
        }
        public POVButton downLeft() {
            return new POVButton(controller, kGameC.dPad.kDownLeft);
        }
        public POVButton left() {
            return new POVButton(controller, kGameC.dPad.kLeft);
        }
        public POVButton upLeft() {
            return new POVButton(controller, kGameC.dPad.kUpLeft);
        }
    }

    public JoystickAxis JoystickAxis = new JoystickAxis();
    public class JoystickAxis{
        public double leftX(){
            return controller.getRawAxis(kGameC.joystickAxis.kLeftX);
        }
        public double leftY(){
            return controller.getRawAxis(kGameC.joystickAxis.kLeftY);
        }
        public double rightX(){
            return controller.getRawAxis(kGameC.joystickAxis.kRightX);
        }
        public double rightY(){
            return controller.getRawAxis(kGameC.joystickAxis.kRightY);
        }
    }

    public TriggerAxis TriggerAxis = new TriggerAxis();
    public  class TriggerAxis{
        public double leftTrigger(){
            return controller.getRawAxis(kGameC.triggers.kLeftT);
        }
        public double RightTrigger(){
            return controller.getRawAxis(kGameC.triggers.kRightT);
        }
    }
    
    
}
