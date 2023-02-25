// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSub extends SubsystemBase {
  public Spark blinkin = new Spark(9);
  int position = 0;
  double[] speeds = {0.91, 0.69, 0.00};
  /** Creates a new LEDSub. */
  public LEDSub() {}

  public void addPos()
  {
    if(position == 2)
    {
      position = 0;
    } else{ position++; }
  }

  public double getSet(){
    return speeds[position];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBlinkIn(double d) {
    blinkin.set(d);
  }
}
