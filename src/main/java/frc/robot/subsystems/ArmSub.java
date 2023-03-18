// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ArmSub extends SubsystemBase {

  int position = 0;
int[] ticksToPos= {0,216,441}; //zero, low, mid, high
int ticksToIntakePos = 69;

  //create falcon 500
WPI_TalonFX pivotMotor = new WPI_TalonFX(10);
WPI_TalonFX pivotMotor2 = new WPI_TalonFX(11);

 WPI_TalonFX extendMotor = new WPI_TalonFX(30);

  /** Creates a new ArmSub. */
  public ArmSub() {
    pivotMotor.set(TalonFXControlMode.PercentOutput, 0);
    pivotMotor2.set(TalonFXControlMode.PercentOutput, 0);

    pivotMotor.setNeutralMode(NeutralMode.Brake);
   pivotMotor2.setNeutralMode(NeutralMode.Brake);
   //pivotMotor.setInverted(true);

    extendMotor.setNeutralMode(NeutralMode.Brake);

    
    extendMotor.setInverted(true);



  }
  // sets pivot motros to  a speed
  public void setPivot(double pwr)
  {
   pivotMotor.set(TalonFXControlMode.PercentOutput, pwr);
   pivotMotor2.set(TalonFXControlMode.PercentOutput, -pwr);

  }
  //sets arm to a speed
  public void setExtend(double pwr)
  {
    extendMotor.set(TalonFXControlMode.PercentOutput, pwr);
  }
  //gets encoder value of arm
  public double getTicks()
  {
    return extendMotor.getSelectedSensorPosition()/100;
  }
  public void resetEncoder(){
    extendMotor.setSelectedSensorPosition(0);
    
  }
public double getPivotTicks(){
  return 0;
  //return pivotMotor.getSelectedSensorPosition();
}
// sees if the arm is fully reatracted


  public void addPos()
  {
    

    if(position == 4){
      position = 1;
    } else {
      if(!(position == 2))
      {
        position++;
      }
    }
  
  }

  public void setPos(int pos)
  {
    position = pos;
  }

  public void subPos()
  {
    if(position == 4){
      position = 0;
    } else {
      if(!(position == 0))
      {
        position--;
      }
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
 

    if(position == 0){
      if(!(getTicks()<8))
      {
        if(getTicks()<ticksToPos[1]){
          setExtend(-0.15);

        } else {
          setExtend(-0.3);

        }
      } else {
        setExtend(0);
      }
    } else if (position == 2){
      if(getTicks() < goalTicks - deadspace){
          setExtend(0.3);
      } else {
        setExtend(0);
      }
    } else if(position ==1) { //pos = 1
      if((getTicks() < goalTicks + deadspace) && (getTicks() > goalTicks - deadspace))
      {
        setExtend(0);
      } else if (getTicks() > (goalTicks)){//above
        setExtend(-0.3);
      } else {
        setExtend(0.3);

      }
    } else if(position ==4){
      if((getTicks() < ticksToIntakePos + (deadspace/2)) && (getTicks() > ticksToIntakePos - deadspace))
      {
        setExtend(0);
      } else if (getTicks() > (goalTicks)){//above
        if(getTicks() < ticksToPos[1])
        {
          setExtend(-0.15);

        } else {
          setExtend(-0.25);

        }
      } else {
        setExtend(0.15);

      }
    } else {
      setExtend(0);

    }




/* 
    //check if in deadspace, only move if in deadspace
   
      if(!(getTicks() > (goalTicks - deadspace) && getTicks() < (goalTicks + deadspace)))
      {
              //when current pos > wanted pos
      if(getTicks() > goalTicks){
        //move backwards, pwr = neg
        if(!getLimitSwitch())
        {

          if(!(getTicks() > (goalTicks - deadspace*3) && getTicks() < (goalTicks + deadspace*3)))
          {
            setExtend(-0.3333*pwr);
          } else {
            setExtend(-pwr);

          }

        }

      } else //when current pos < wanted pos
      {
        //move forwards, pwr = pos
        if(!(getTicks() > (goalTicks - deadspace*2) && getTicks() < (goalTicks + deadspace*2)))
          {
            setExtend(pwr);
          }else{
            setExtend(pwr);

          }

      }
      } else {
        setExtend(0);
      }


    

*/
    

    
  }
  

  public void xboxPivot(double y) {

    if(y > 0.9){
//+
      setPivot(0.135);
    } else if (y < -0.9)
    {
      //-
      setPivot(-0.135);

    } else {
      setPivot(0);
    }
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getPosition() {
    return position;
  }

}
