// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class RollerSub extends SubsystemBase{
    // making motors and encoder

    private static CANSparkMax neoR  = new CANSparkMax(21,  MotorType.kBrushless);

    public RollerSub(){
        // setting stuff up
        
 
    }
    // get encoder
    public double getEncoder(){
       // neoREN = neoR.getEncoder();
        //return neoREN.getPosition();
        return 0;
    }
    // set speed of roller
    public void setRoller(double pwr){
        neoR.set(pwr);
        System.out.println(pwr);
      //  neoREN = neoR.getEncoder();
    }

}
