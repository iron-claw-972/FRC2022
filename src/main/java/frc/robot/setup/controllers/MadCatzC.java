package frc.robot.setup.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.constants.controller.kMadCatzC;

public class MadCatzC {

    // System.out.println("thing");
    public static class Button {

        public static JoystickButton b1(Joystick controller) {
            return new JoystickButton(controller, kMadCatzC.buttons.k1);
        }
        public static JoystickButton b2(Joystick controller) {
            return new JoystickButton(controller, kMadCatzC.buttons.k2);
        }
        public static JoystickButton b3(Joystick controller) {
            return new JoystickButton(controller, kMadCatzC.buttons.k3);
        }
        public static JoystickButton b4(Joystick controller) {
            return new JoystickButton(controller, kMadCatzC.buttons.k4);
        }
        public static JoystickButton b5(Joystick controller) {
            return new JoystickButton(controller, kMadCatzC.buttons.k5);
        }
        public static JoystickButton b6(Joystick controller) {
            return new JoystickButton(controller, kMadCatzC.buttons.k6);
        }
        public static JoystickButton b7(Joystick controller) {
            return new JoystickButton(controller, kMadCatzC.buttons.k7);
        }
    }
    public static class DPad {
        public static POVButton up(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kUp);
        }
        public static POVButton upRight(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kUpRight);
        }
        public static POVButton right(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kRight);
        }
        public static POVButton downRight(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kDownRight);
        }
        public static POVButton down(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kDown);
        }
        public static POVButton downLeft(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kDownLeft);
        }
        public static POVButton left(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kLeft);
        }
        public static POVButton upLeft(Joystick controller) {
            return new POVButton(controller, kMadCatzC.thumbstick.kUpLeft);
        }
    }
    public static class joystickAxis{
        public static double X(Joystick controller){
            return controller.getRawAxis(kMadCatzC.joystickAxis.kX);
        }
        public static double Y(Joystick controller){
            return controller.getRawAxis(kMadCatzC.joystickAxis.kY);
        }
        public static double zRotate(Joystick controller){
            return controller.getRawAxis(kMadCatzC.joystickAxis.kZRotate);
        }
        public static double zAxis(Joystick controller){
            return controller.getRawAxis(kMadCatzC.joystickAxis.kZAxis);
        }
    }
}
