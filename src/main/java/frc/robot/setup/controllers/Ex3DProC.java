package frc.robot.setup.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.constants.controller.kEx3DProC;

public class Ex3DProC extends Controller{
    
    public Ex3DProC(Joystick joystick_){
        super(joystick_);
    }
    
    //returns JoystickButton object
    Button Button = new Button();
    public class Button {
        public JoystickButton b1() {
            return new JoystickButton(controller, kEx3DProC.buttons.k1);
        }
        public JoystickButton b2() {
            return new JoystickButton(controller, kEx3DProC.buttons.k2);
        }
        public JoystickButton b3() {
            return new JoystickButton(controller, kEx3DProC.buttons.k3);
        }
        public JoystickButton b4() {
            return new JoystickButton(controller, kEx3DProC.buttons.k4);
        }
        public JoystickButton b5() {
            return new JoystickButton(controller, kEx3DProC.buttons.k5);
        }
        public JoystickButton b6() {
            return new JoystickButton(controller, kEx3DProC.buttons.k6);
        }
        public JoystickButton b7() {
            return new JoystickButton(controller, kEx3DProC.buttons.k7);
        }
        public JoystickButton b8() {
            return new JoystickButton(controller, kEx3DProC.buttons.k8);
        }
        public JoystickButton b9() {
            return new JoystickButton(controller, kEx3DProC.buttons.k9);
        }
        public JoystickButton b10() {
            return new JoystickButton(controller, kEx3DProC.buttons.k10);
        }
        public JoystickButton b11() {
            return new JoystickButton(controller, kEx3DProC.buttons.k11);
        }
        public JoystickButton b12() {
            return new JoystickButton(controller, kEx3DProC.buttons.k12);
        }
    }

    //returns Joystick Axis value
    public JoystickAxis JoystickAxis = new JoystickAxis();
    public class JoystickAxis{
        public double X(){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kX);
        }
        public double Y(){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kY);
        }
        public double Z(){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kZ);
        }
        public double slider(){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kSlider);
        }
    }


}
