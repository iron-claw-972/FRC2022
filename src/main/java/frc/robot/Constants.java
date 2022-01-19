/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
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
		public static final double kJoystickDeadband = 0.07; // How much of joystick is "dead" zone [0,1]
    }

    public final class kDrive {
        public static final int kLeftMotorPort = 0;
        public static final int kLeftMotorPalPort = -1;

        public static final int kRightMotorPort = 1;
        public static final int kRightMotorPalPort = -1;
    }
}
