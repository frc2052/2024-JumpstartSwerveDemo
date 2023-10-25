// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Drivetrain {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_ENCODER = 5;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = Math.toRadians(217.6);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_ENCODER = 8;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = Math.toRadians(175.7);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_ENCODER = 2;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS = Math.toRadians(39.9);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = Math.toRadians(61.1);
  }
}
