// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSub extends SubsystemBase {

  //create falcon 500
  WPI_TalonFX pivotMotor = new WPI_TalonFX(6);
  WPI_TalonFX extendMotor = new WPI_TalonFX(7);

  /** Creates a new ArmSub. */
  public ArmSub() {

    pivotMotor.setNeutralMode(NeutralMode.Brake);
    extendMotor.setNeutralMode(NeutralMode.Brake);



  }

  public void setPivot(double pwr)
  {
    pivotMotor.set(TalonFXControlMode.PercentOutput, pwr);
  }

  public void setExtend(double pwr)
  {
    extendMotor.set(TalonFXControlMode.PercentOutput, pwr);
  }

  public double getTicks()
  {
    return extendMotor.getSelectedSensorPosition();
  }



  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
