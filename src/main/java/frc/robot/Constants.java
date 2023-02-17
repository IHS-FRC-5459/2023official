// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
      public static final double driveWheelCirc = 0.1524 * Math.PI;//in m 
      public static final double encoderResolution = 1; //  actually 42
      private static final double driveRatio = 9.643/1; // needs to be affirmed for 2nd time // Act 9.643/1
      public static final double distancePerPulse = driveWheelCirc * (1/(driveRatio * /*encoderResolution*/1));
      
      
      public static final int distForArmToExToGetConeInTicks = 1000; // needs to be correct


      //robot drive characterization
      public final static double drivetrain_kA = 0.12; // I think needs to be correct

      //arm to level distances in ticks 
      public final static double ticksToMid = 500; //needs to be correct
      public final static double ticksToHigh = 1000; //needs to be correct
      public final static double ticksToCone = 750; // needs to be correct
      //drivtrain char
      public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kTrackwidthMeters = 0.6096;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kPDriveVel = 8.5;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    


    }
