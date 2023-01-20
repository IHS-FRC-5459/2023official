// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSub extends SubsystemBase {
  //create neo
  private static CANSparkMax neoClaw;
  private static RelativeEncoder neoClawEN;
  /** Creates a new ClawSub. */
  public ClawSub() {
    neoClaw = new CANSparkMax(5, MotorType.kBrushless);
    neoClaw.setInverted(false);
  }
  // sets power of claw motor
  public void setClaw(double pwr)
  {
    neoClaw.set(pwr);
    neoClawEN = neoClaw.getEncoder();
  }
  // gets encoder value of claw
  public double getEncoder()
  {
    neoClawEN = neoClaw.getEncoder();
    return neoClawEN.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
