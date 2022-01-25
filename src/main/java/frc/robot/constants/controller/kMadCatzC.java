package frc.robot.constants.controller;

public final class kMadCatzC {
    public final class buttons {
        // Trigger Button
        public static final int k1 = 1;

        // 5 Buttons on Top Front of Joystick
        public static final int k2 = 2;
        public static final int k3 = 3;
        public static final int k4 = 4;
        public static final int k5 = 5;
        public static final int k6 = 6;

        // Button on Middle Back of Joystick
        public static final int k7 = 7;
    }

    public final class joystickAxis {
        // Joystick axis, X is Right/Left, Y is Up/Down, Z is twist Right/Left
        public static final int kX = 0;
        public static final int kY = 1;
        public static final int kZRotate = 3;
        // Slider on Front Base of Controller
        public static final int kZAxis = 2;
    }

    public final class thumbstick {
        // 8 Directional Inputs on Hat Switch
        public static final int kUp = 0;
        public static final int kUpRight = 45;
    
        public static final int kRight = 90;
        public static final int kDownRight = 135;

        public static final int kDown = 180;
        public static final int kDownLeft = 235;

        public static final int kLeft = 270;
        public static final int kUpLeft = 315;
    }
}