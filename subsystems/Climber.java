// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  static CANSparkMax leftClimber;
  static CANSparkMax rightClimber;
  public static boolean brakeCheck = true;
 
  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new CANSparkMax(15, MotorType.kBrushless);
    leftClimber.setInverted(false);
    rightClimber = new CANSparkMax(16, MotorType.kBrushless);
    rightClimber.setInverted(false);
  }

  @Override
  public void periodic() {
    
    if (brakeCheck == true) {
      rightClimber.setIdleMode(IdleMode.kBrake);
      leftClimber.setIdleMode(IdleMode.kBrake);
      
    } else {
      rightClimber.setIdleMode(IdleMode.kCoast);
      leftClimber.setIdleMode(IdleMode.kCoast);
      System.out.println("cool");
    }
    // // This method will be called once per scheduler run
  }
  

  public void LeftClimb(double speed){
    leftClimber.set(speed);
  }

  public void RightClimb(double speed){
    rightClimber.set(speed);
  }
  
  public void stop(){
    leftClimber.set(0);
    rightClimber.set(0);
  }

}
