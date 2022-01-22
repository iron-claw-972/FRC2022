package frc.robot.utils.controlers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class Game {

    // System.out.println("thing");
    public static class Button {

        public static JoystickButton A(Joystick controller) {
            return new JoystickButton(controller, 1);
        }
        public static JoystickButton B(Joystick controller) {
            return new JoystickButton(controller, 2);
        }
        public static JoystickButton X(Joystick controller) {
            return new JoystickButton(controller, 3);
        }
        public static JoystickButton Y(Joystick controller) {
            return new JoystickButton(controller, 4);
        }
        public static JoystickButton LB(Joystick controller) {
            return new JoystickButton(controller, 5);
        }
        public static JoystickButton RB(Joystick controller) {
            return new JoystickButton(controller, 6);
        }
        public static JoystickButton BACK(Joystick controller) {
            return new JoystickButton(controller, 7);
        }
        public static JoystickButton START(Joystick controller) {
            return new JoystickButton(controller, 8);
        }
    }
    public static class DPad {
        public static POVButton up(Joystick controller) {
            return new POVButton(controller, 0);
        }
        // UP = new POVButton(driver, 0),
    // RIGHT = new POVButton(driver, 90),
    // DOWN = new POVButton(driver, 180),
    // LEFT = new POVButton(driver, 270);
    }
    public static class joystickAxis{
        public static double leftX(Joystick controller){
            return controller.getRawAxis(0);
        }
    }
    
    
}
