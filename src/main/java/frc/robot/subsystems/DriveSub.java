// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSub extends SubsystemBase {

 
  // created motors
  private CANSparkMax neo1, neo2, neo3, neo4;
  // encoders
  private RelativeEncoder leftEN, rightEN;
  private Encoder driveEnc;
  //IMU unit
  // used to see roll, yaw, and pitch of the robot. Used for leveling, and truning. Basically like a gyro.
  private Pigeon2 imu;


  
  /** Creates a new DriveSub. */
  public DriveSub() {
    //set motor id's 
    neo1 = new CANSparkMax(1, MotorType.kBrushless);
    neo2 = new CANSparkMax(2, MotorType.kBrushless);
    neo3 = new CANSparkMax(3, MotorType.kBrushless);
    neo4 = new CANSparkMax(4, MotorType.kBrushless);

    //set right side motors inverted
    neo3.setInverted(true);
    neo4.setInverted(true);

    //instatiate pigeon IMU   make pigeon actually do stuff    
    imu = new Pigeon2(0);

    //motor encoders
    leftEN = neo1.getEncoder();
    rightEN = neo4.getEncoder();

    driveEnc = new Encoder(5,6,true,EncodingType.k4X);
    addChild("driveEnc", driveEnc);
    driveEnc.setDistancePerPulse(Constants.distancePerPulse);



  }

  //sets drivebase speeds - by side l/r
  public void setDrive(double left, double right)
  {
    neo1.set(left);
    neo2.set(left);
    neo3.set(right);
    neo4.set(right);
  }

  //get distance travelled
  public double getDistance()
  {
      //get encoders
      //double leftDistance = leftEN.getPosition();
      //double rightDistance = rightEN.getPosition();
      //average position
      //double averagePosition = (leftDistance + rightDistance)/2;

      //convert encoder counts to distance.
      //double dist = averagePosition/Constants.distancePerPulse;
      double dist = driveEnc.getDistance();
      return dist;//in inches
  }



  //get angle  // left and right
  public double getYaw()
  {
    double yaw = imu.getYaw();
    return yaw;

  }

  //get roll // up/down
  public double getRoll()
  {
    double roll = imu.getRoll();
    return roll;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
