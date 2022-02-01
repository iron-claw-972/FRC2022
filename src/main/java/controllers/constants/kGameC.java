package controllers.constants;

public final class kGameC {
    public static final class buttons {
        // 4 Buttons on Right of Controller
        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;

        // 2 Bumpers on Top of Controller, Back and Start Buttons on Center of Front of Controller
        public static final int kLB = 5;
        public static final int kRB = 6;
        public static final int kBack = 7;
        public static final int kStart = 8;

        // Pushing down on Left and Right Joystick
        public static final int kLeftJ = 9;
        public static final int kRightJ = 10;
    }

    public static final class dPad {
        // 8 D-Pad Buttons on left of Controller
        public static final int kUp = 0;
        public static final int kUpRight = 45;
    
        public static final int kRight = 90;
        public static final int kDownRight = 135;

        public static final int kDown = 180;
        public static final int kDownLeft = 235;

        public static final int kLeft = 270;
        public static final int kUpLeft = 315;
    }

    public static final class joystickAxis {
        // Left Joystick axis, X is Right/Left and Y is Up/Down
        public static final int kLeftX = 0;
        public static final int kLeftY = 1;
        // Right Joystick axis, X is Right/Left and Y is Up/Down
        public static final int kRightX = 4;
        public static final int kRightY = 5;
    }

    public static final class triggers {
        // Left and Right Triggers on Controller
        public static final int kLeftT = 2;
        public static final int kRightT = 3;
    }
}
