// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ClawSub extends SubsystemBase {

  CANSparkMax neoClaw = new CANSparkMax(15, MotorType.kBrushless);

  //create neo
  private static RelativeEncoder neoClawEN;
  /** Creates a new ClawSub. */
  public ClawSub() {
    neoClawEN = neoClaw.getEncoder();
   ////neoClaw = new CANSparkMax(18, MotorType.kBrushless);
   //neoClaw.setInverted(false);
  }
  // sets power of claw motor
  public void setClaw(double pwr)
  {
   neoClaw.set(pwr);
   System.out.println(pwr);
   // neoClawEN = neoClaw.getEncoder();
  }
  // gets encoder value of claw
  public double getEncoder()
  {//return 0;
   return neoClawEN.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void moveClaw(double xboxLeftTrigger, double xboxRightTrigger) {
  if(xboxRightTrigger == 2){
    setClaw(0.4);

  } else {
    if(xboxLeftTrigger < 0.1 && xboxRightTrigger < 0.1)
    {
      setClaw(0);
    } else 
    {
      if(xboxLeftTrigger > 0.1)
      {
        setClaw(-0.3);
        Robot.setClawConstant(false);
      } else {
        if(xboxRightTrigger > 0.5){
          setClaw(0.4);
    
        } else if (xboxRightTrigger > 0.1) {
          setClaw(0.3);
    
        }
      }
    }
  }
  
 

}
}

