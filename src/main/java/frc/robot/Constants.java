// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // drivetrain IDs
    public static int leftFrontMotorPort = 1;
    public static int leftBackMotorPort = 4;
    public static int rightFrontMotorPort = 2;
    public static int rightBackMotorPort = 5;

    // elevator IDs
    public static int elevatorRightMotorPort = 7;
    public static int elevatorLeftMotorPort = 6;
    public static int limitPort = 3;
    public static int shooterPort = 8;
    // TODO: Configure motors/sensors for these ports

    // Configured these values in inches
    public static int LEVEL_0_HEIGHT = 8;
    public static int LEVEL_1_HEIGHT = 8;
    public static int LEVEL_2_HEIGHT = 8;
    public static int LEVEL_3_HEIGHT = 8;

    // general
    public static double gearRatio = 10.86;
    public static double wheelRadius = 3;
    public static int encoderTicksPerRotation = 42;

    public static boolean slowMode = false;
    public static double slowModeMultipler = 0.3;
    /**
     * Determines the number of encoder ticks necessary for drivetrain to turn at certain angle
     * @param angle the angle to turn
     * @return number of encoder ticks for the each side of the drivetrain to turn
     */
    public static double rotateX(double angle){
      double encoderTicks = ((13/90)* angle);
      return encoderTicks;
    }

    public static final double allowedError = 0.05;
    public static final int leftAxis = 1;
    public static final int rightAxis = 5;
}