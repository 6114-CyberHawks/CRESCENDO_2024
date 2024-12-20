// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  CANSparkMax topMotor;
  CANSparkMax bottomMotor;
 
  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new CANSparkMax(13, MotorType.kBrushless);
    topMotor.setInverted(true);
    bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
    bottomMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public void ShootSpeaker(double speed){
    topMotor.set(speed);
    bottomMotor.set(speed);
  }
  
  public void stop(){
    topMotor.set(0);
    bottomMotor.set(0);
  }

}
