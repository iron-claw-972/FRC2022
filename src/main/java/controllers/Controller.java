package controllers;
import edu.wpi.first.wpilibj.Joystick;

public class Controller {

    //for storeing the controller object
    public Joystick controller;
    
    //gets controller object used for return methods
    public Controller(Joystick joystick_){
        controller = joystick_;
    }
}
