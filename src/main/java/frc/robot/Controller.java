package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.kJoy;

public class Controller {
    static Joystick driver = new Joystick(kJoy.kDriverJoy);
    static Joystick operator = new Joystick(kJoy.kOperatorJoy);

    private static final JoystickButton driver_A = new JoystickButton(driver, 1),
    driver_B = new JoystickButton(driver, 2), driver_X = new JoystickButton(driver, 3),
    driver_Y = new JoystickButton(driver, 4), driver_LB = new JoystickButton(driver, 5),
    driver_RB = new JoystickButton(driver, 6), driver_BACK = new JoystickButton(driver, 7),
    driver_START = new JoystickButton(driver, 8),

    driver_stickButt = new JoystickButton(driver, 1);

  private static final JoystickButton operator_A = new JoystickButton(operator, 1),
    operator_B = new JoystickButton(operator, 2), operator_X = new JoystickButton(operator, 3),
    operator_Y = new JoystickButton(operator, 4), operator_LB = new JoystickButton(operator, 5),
    operator_RB = new JoystickButton(operator, 6), operator_BACK = new JoystickButton(operator, 7),
    operator_START = new JoystickButton(operator, 8);

  private static final POVButton driver_DPAD_UP = new POVButton(driver, 0),
    driver_DPAD_RIGHT = new POVButton(driver, 90), driver_DPAD_DOWN = new POVButton(driver, 180),
    driver_DPAD_LEFT = new POVButton(driver, 270);

  private static final POVButton operator_DPAD_UP = new POVButton(operator, 0),
    operator_DPAD_RIGHT = new POVButton(driver, 90), operator_DPAD_DOWN = new POVButton(operator, 180),
    operator_DPAD_LEFT = new POVButton(driver, 270);

  public Controller(){
    
  }
  
    public void configureButtonBindings() {
    // operator_B.whenPressed(() -> m_drive.modSensitivity());
    driver_A.whenPressed(() -> m_drive.modDrive());
    // driver_stickButt.whenPressed(() -> m_drive.modDrive());
  }
}
