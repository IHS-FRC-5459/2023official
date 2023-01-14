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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

      //drive base encoder information
      public static final double driveWheelCirc = 6 * Math.PI;
      public static final double encoderResolution = 42;
      private static final double driveRatio = 6/1;
      public static final double distancePerPulse = driveWheelCirc * (1/(driveRatio * encoderResolution));



      //robot drive characterization
      public final static double drivetrain_kA = 0.12;

      //arm to level distances in ticks 
      public final static double ticksToMid = 500;
      public final static double ticksToHigh = 1000;
}
