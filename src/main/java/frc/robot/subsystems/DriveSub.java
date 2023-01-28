// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class DriveSub extends SubsystemBase {
  private final MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(
    new WPI_TalonSRX(0),
    new WPI_TalonSRX(1)
  );
  private final MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(
    new WPI_TalonSRX(3),
    new WPI_TalonSRX(4)
  );
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final Encoder m_leftEncoder =
  new Encoder(5,6,true,EncodingType.k4X); // 5 6
  private final Encoder m_rightEncoder =
  new Encoder(2,3,false,EncodingType.k4X); // 2 3

  private final Gyro m_gyro = new ADXRS450_Gyro();
  private Pigeon2 m_imu = new Pigeon2(5);
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSub. */
  public DriveSub() {
    m_rightMotors.setInverted(true);
    m_leftEncoder.setDistancePerPulse(Constants.distancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.distancePerPulse);
    
    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void setDrive(double left, double right)
  {
    m_leftMotors.set(left);
    m_rightMotors.set(right);


    
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }


  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public double getPitch()
  {
    return m_imu.getPitch();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    
    return new SequentialCommandGroup(
     new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      if(isFirstPath){
          this.resetOdometry(traj.getInitialPose());
      }
    }),

    new PPRamseteCommand(
      traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(Constants.kPDriveVel, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.kPDriveVel, 0, 0), // Right controller (usually the same values as left controller)
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    
    );

    
}

}
