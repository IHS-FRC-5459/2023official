// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase {

  int position = 0;
  int[] ticksToPos= {0,100,200,300}; //zero, low, mid, high

  //create falcon 500
WPI_TalonFX pivotMotor = new WPI_TalonFX(10);
WPI_TalonFX pivotMotor2 = new WPI_TalonFX(11);

 WPI_TalonFX extendMotor = new WPI_TalonFX(30);
 DigitalInput bottomLimit = new DigitalInput(0);

  /** Creates a new ArmSub. */
  public ArmSub() {
    pivotMotor.set(TalonFXControlMode.PercentOutput, 0);
    pivotMotor2.set(TalonFXControlMode.PercentOutput, 0);

    pivotMotor.setNeutralMode(NeutralMode.Brake);
   pivotMotor2.setNeutralMode(NeutralMode.Brake);
   //pivotMotor.setInverted(true);

  //  extendMotor.setNeutralMode(NeutralMode.Brake);



  }
  // sets pivot motros to  a speed
  public void setPivot(double pwr)
  {
   pivotMotor.set(TalonFXControlMode.PercentOutput, pwr);
   pivotMotor2.set(TalonFXControlMode.PercentOutput, -pwr);

  }
// sets arm to a speed
  public void setExtend(double pwr)
  {
    extendMotor.set(TalonFXControlMode.PercentOutput, pwr);
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
  { 
    return bottomLimit.get();
  }


  public void addPos()
  {
    if(!(position == 4))
    {
      position++;
    }
  
  }

  public void subPos()
  {
    if(!(position == 0))
    {
      position--;
    }
  }

  public double getRequiredTicks()
  {
    return ticksToPos[position];
  }

  public void moveToPosition(double pwr, int deadspace)
  {
    // if touching limit switch, set encoder value to 0 and dont allow going backwards
    // if current ticks > goalTicks, move with -pwr to current ticks = goalticks +- deadspace
    // if current ticks < goalTicks, move with pwr to current ticks = goalticks +- deadspace
    int goalTicks = ticksToPos[position];

    //reset enc
    if(getLimitSwitch())
    {
      resetEncoder();
    }

    //check if in deadspace, only move if in deadspace
    if(!(getTicks() > (goalTicks - deadspace) && getTicks() < (goalTicks + deadspace)))
    {
      //when current pos > wanted pos
      if(getTicks() > goalTicks){
        //move backwards, pwr = neg
        if(!getLimitSwitch())
        {
          setExtend(-pwr);

        }

      } else //when current pos < wanted pos
      {
        //move forwards, pwr = pos
        setExtend(pwr);

      }

    }


    

    
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
