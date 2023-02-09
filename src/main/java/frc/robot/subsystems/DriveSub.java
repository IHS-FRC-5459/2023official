// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import org.ejml.dense.block.decomposition.hessenberg.TridiagonalDecompositionHouseholder_MT_FDRB;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  CANSparkMax bottomLeft = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax topLeft = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax bottomRight =new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax topRight = new CANSparkMax(5, MotorType.kBrushless);

  
  RelativeEncoder topLeftEncoder;
  RelativeEncoder topRightEncoder;
  private final DifferentialDrive m_drive = new DifferentialDrive(bottomLeft, bottomRight);




  private final Gyro m_gyro = new ADXRS450_Gyro();
  //private Pigeon2 m_imu = new Pigeon2(6);
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSub. */
  public DriveSub() {
    bottomLeft.follow(topLeft);
    bottomRight.follow(topRight);
    bottomRight.setInverted(true);
    topRight.setInverted(true);
    topLeftEncoder = topLeft.getEncoder();
    topRightEncoder = topRight.getEncoder();
    topLeftEncoder.setPositionConversionFactor(Constants.distancePerPulse);
    topRightEncoder.setPositionConversionFactor(Constants.distancePerPulse);
    topLeftEncoder.setVelocityConversionFactor(Constants.distancePerPulse);
    topRightEncoder.setVelocityConversionFactor(Constants.distancePerPulse);

    
    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), topLeftEncoder.getPosition(), topRightEncoder.getPosition());
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(topLeftEncoder.getVelocity(), topRightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), topLeftEncoder.getPosition(), topRightEncoder.getPosition(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    topLeft.setVoltage(leftVolts);
    topRight.setVoltage(rightVolts);
    //m_drive.feed();
  }

  public void setDrive(double left, double right)
  {
    topLeft.set(left);
    topRight.set(right);


    
  }

  public void resetEncoders() {
    topLeftEncoder.setPosition(0);
    topRightEncoder.setPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (topLeftEncoder.getPosition() + topRightEncoder.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return topLeftEncoder;
  }


  public RelativeEncoder getRightEncoder() {
    return topRightEncoder;
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
    return 0;
    //return m_imu.getPitch();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), topLeftEncoder.getPosition(), topRightEncoder.getPosition());
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) throws NullPointerException{
    
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
