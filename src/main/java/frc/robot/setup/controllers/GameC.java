package frc.robot.setup.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.constants.controller.*;

public class GameC {

    public static class Button {

        public static JoystickButton A(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kA);
        }
        public static JoystickButton B(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kB);
        }
        public static JoystickButton X(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kX);
        }
        public static JoystickButton Y(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kY);
        }
        public static JoystickButton LB(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kLB);
        }
        public static JoystickButton RB(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kRB);
        }
        public static JoystickButton back(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kBack);
        }
        public static JoystickButton start(Joystick controller) {
            return new JoystickButton(controller, kGameC.buttons.kStart);
        }
    }
 
    public static class DPad {
        public static POVButton up(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kUp);
        }
        public static POVButton upRight(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kUpRight);
        }
        public static POVButton right(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kRight);
        }
        public static POVButton downRight(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kDownRight);
        }
        public static POVButton down(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kDown);
        }
        public static POVButton downLeft(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kDownLeft);
        }
        public static POVButton left(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kLeft);
        }
        public static POVButton upLeft(Joystick controller) {
            return new POVButton(controller, kGameC.dPad.kUpLeft);
        }
    }

    public static class JoystickAxis{
        public static double leftX(Joystick controller){
            return controller.getRawAxis(kGameC.joystickAxis.kLeftX);
        }
        public static double leftY(Joystick controller){
            return controller.getRawAxis(kGameC.joystickAxis.kLeftY);
        }
         public static double RightX(Joystick controller){
            return controller.getRawAxis(0);
        }
        public static double RightY(Joystick controller){
            return controller.getRawAxis(0);
        }
    }

    public static class TriggerAxis{
        public static double leftTrigger(Joystick controller){
            return controller.getRawAxis(kGameC.triggers.kLeftT);
        }
        public static double RightTrigger(Joystick controller){
            return controller.getRawAxis(0);
        }
    }
    
    
}
