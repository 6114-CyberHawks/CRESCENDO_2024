// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax leftIntake;
  CANSparkMax rightIntake;
  CANSparkMax middleMotor;
 
  /** Creates a new Intake. */
  public Intake() {
    leftIntake = new CANSparkMax(11, MotorType.kBrushless);
    leftIntake.setInverted(false);
    rightIntake = new CANSparkMax(12, MotorType.kBrushless);
    rightIntake.setInverted(false);
    middleMotor = new CANSparkMax(18, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public void intakeRun(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void prepareNote(double speed) {
    middleMotor.set(speed);
  }


  public void dropObject(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void stop(){
    leftIntake.set(0);
    rightIntake.set(0);
    middleMotor.set(0);
  }

}
