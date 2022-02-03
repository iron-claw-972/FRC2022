package controllers;

import controllers.constants.Ex3DProConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class Ex3DProController extends Controller{
    
  private Button Button = new Button();
  private JoystickAxis JoystickAxis = new JoystickAxis();

    public Ex3DProController(Joystick joystick_){
        super(joystick_);
    }
    
    public JoystickAxis getJoystickAxis() {
        return JoystickAxis;
    }

    public Button getButton() {
        return Button;
    }

    //returns JoystickButton object
    public class Button {
      public JoystickButton b1() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k1);
      }
      public JoystickButton b2() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k2);
      }
      public JoystickButton b3() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k3);
      }
      public JoystickButton b4() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k4);
      }
      public JoystickButton b5() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k5);
      }
      public JoystickButton b6() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k6);
      }
      public JoystickButton b7() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k7);
      }
      public JoystickButton b8() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k8);
      }
      public JoystickButton b9() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k9);
      }
      public JoystickButton b10() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k10);
      }
      public JoystickButton b11() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k11);
      }
      public JoystickButton b12() {
          return new JoystickButton(getController(), Ex3DProConstants.buttons.k12);
      }
    }

    //returns Joystick Axis value
    public class JoystickAxis{
      public double X(){
          return getController().getRawAxis(Ex3DProConstants.joystickAxis.kX);
      }
      public double Y(){
          return getController().getRawAxis(Ex3DProConstants.joystickAxis.kY);
      }
      public double Z(){
          return getController().getRawAxis(Ex3DProConstants.joystickAxis.kZ);
      }
      public double slider(){
          return getController().getRawAxis(Ex3DProConstants.joystickAxis.kSlider);
      }
  }
  
}
