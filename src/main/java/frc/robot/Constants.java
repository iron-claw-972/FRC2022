/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * 
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static final double kMaxVoltage = 12.0;

    public final class kJoy {
        public static final int kDriverJoy = 0;
        public static final int kOperatorJoy = 1;
		public static final double kJoystickDeadband = 0.00; // How much of joystick is "dead" zone [0,1]

        
    }

    public final class kDrive {
        public static final int kLeftMotorPort = 1;
        // public static final int kLeftMotorPalPort = -1;

        public static final int kRightMotorPort = 2;
        // public static final int kRightMotorPalPort = -1;
    }
//C stands for Controller
    public final class kGameC {
        public final class buttons {
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

        public final class dPad {
            // 8 D-Pad Buttons on left of Controller
            public static final int kUp = 0;
            public static final int kUpRight = 45;
        
            public static final int kRight = 90;
            public static final int kDownRight = 135;

            public static final int kDown = 180;
            public static final int kDownleft = 235;

            public static final int kLeft = 270;
            public static final int kUpLeft = 315;
        }

        public final class joystickAxis {
            // Left Joystick axis, X is Right/Left and Y is Up/Down
            public static final int kLeftX = 0;
            public static final int kLeftY = 1;
            // Right Joystick axis, X is Right/Left and Y is Up/Down
            public static final int kRightX = 4;
            public static final int kRightY = 5;
        }

        public final class triggers {
            // Left and Right Triggers on Controller
            public static final int kLeftT = 2;
            public static final int kRightT = 3;
        }
    }

    public final class kEx3DProC {
        public final class buttons {
            // Trigger Button
            public static final int k1 = 1;

            // Thumb Button on left of Joystick
            public static final int k2 = 2;
            
            // 4 Buttons on Top of Joystick
            public static final int k3 = 3;
            public static final int k4 = 4;
            public static final int k5 = 5;
            public static final int k6 = 6;

            // 6 Buttons on Left Base of Controller
            public static final int k7 = 7;
            public static final int k8 = 8;
            public static final int k9 = 9;
            public static final int k10 = 10;
            public static final int k11 = 11;
            public static final int k12 = 12;
        }

        public final class joystickAxis {
            // Joystick axis, X is Right/Left, Y is Up/Down, Z is twist Right/Left
            public static final int kX = 0;
            public static final int kY = 1;
            public static final int kZRotate = 2;
            // Slider on Front Base of Controller
            public static final int kSlider = 3;
        }
    }
    
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
            // 8 Directional Inputs on Thumbstick
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
}
