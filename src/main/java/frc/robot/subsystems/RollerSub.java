// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class RollerSub extends SubsystemBase{
    // making motors and encoder

    private static CANSparkMax neoR;
    private static RelativeEncoder neoREN;

    public RollerSub(){
        // setting stuff up
        neoR = new CANSparkMax(7,  MotorType.kBrushless);
        neoR.setInverted(true);
    }
    // get encoder
    public double getEncoder(){
        neoREN = neoR.getEncoder();
        return neoREN.getPosition();
    }
    // set speed of roller
    public void setRoller(double pwr){
        neoR.set(pwr);
        neoREN = neoR.getEncoder();
    }

}