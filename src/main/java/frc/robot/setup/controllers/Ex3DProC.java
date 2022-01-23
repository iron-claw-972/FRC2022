package frc.robot.setup.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.constants.controller.kEx3DProC;

public class Ex3DProC {
    public static class Button {

        public static JoystickButton b1(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k1);
        }
        public static JoystickButton b2(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k2);
        }
        public static JoystickButton b3(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k3);
        }
        public static JoystickButton b4(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k4);
        }
        public static JoystickButton b5(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k5);
        }
        public static JoystickButton b6(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k6);
        }
        public static JoystickButton b7(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k7);
        }
        public static JoystickButton b8(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k8);
        }
        public static JoystickButton b9(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k9);
        }
        public static JoystickButton b10(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k10);
        }
        public static JoystickButton b11(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k11);
        }
        public static JoystickButton b12(Joystick controller) {
            return new JoystickButton(controller, kEx3DProC.buttons.k12);
        }
    }

    public static class JoystickAxis{
        public static double X(Joystick controller){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kX);
        }
        public static double Y(Joystick controller){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kY);
        }
        public static double Z(Joystick controller){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kZ);
        }
        public static double slider(Joystick controller){
            return controller.getRawAxis(kEx3DProC.joystickAxis.kSlider);
        }
    }


}
