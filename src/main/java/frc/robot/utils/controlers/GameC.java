package frc.robot.utils.controlers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants;
import frc.robot.Constants.kGameC;

public class GameC {

    // System.out.println("thing");
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
