// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase {

  //create falcon 500
  //WPI_TalonFX pivotMotor = new WPI_TalonFX(6);
 //WPI_TalonFX extendMotor = new WPI_TalonFX(7);
 // DigitalInput bottomLimit = new DigitalInput(0);

  /** Creates a new ArmSub. */
  public ArmSub() {

   // pivotMotor.setNeutralMode(NeutralMode.Brake);
    //extendMotor.setNeutralMode(NeutralMode.Brake);



  }
  // sets pivot motros to  a speed
  public void setPivot(double pwr)
  {
   // pivotMotor.set(TalonFXControlMode.PercentOutput, pwr);
  }
// sets arm to a speed
  public void setExtend(double pwr)
  {
   // extendMotor.set(TalonFXControlMode.PercentOutput, pwr);
  }
// gets encoder value of arm
  public double getTicks()
  {
    return 0;  //return extendMotor.getSelectedSensorPosition();
  }
  public void resetEncoder(){
    //extendMotor.setSelectedSensorPosition(0);
    
  }
public double getPivotTicks(){
  return 0;
  //return pivotMotor.getSelectedSensorPosition();
}
// sees if the arm is fully reatracted
  public boolean getLimitSwitch()
  { return false;
   // return bottomLimit.get();
  }



  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
