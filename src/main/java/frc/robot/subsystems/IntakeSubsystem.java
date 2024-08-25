// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class IntakeSubsystem extends SubsystemBase {


  public TalonFX IntakeRoller;
  public TalonFX GroundIntake;

  public double reset;
  public TalonFX Shooter;

  public CoreCANcoder ShootercanCoder;
  public double conert = 100;
  public TalonFX Feeder;


  public static DigitalInput BeamBreakShooter;


  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
  IntakeRoller = new TalonFX(14);
  
  Shooter = new TalonFX(100);  
  GroundIntake= new TalonFX(15);

  Feeder = new TalonFX(16);
  


  ShootercanCoder = new CANcoder(100);


  BeamBreakShooter = new DigitalInput(0);











  Shooter.getConfigurator().apply(new TalonFXConfiguration());  



  Shooter.setInverted(false);
  TalonFXConfiguration ShooterConfigs = new TalonFXConfiguration();

  ShooterConfigs.MotorOutput.withPeakForwardDutyCycle(0.5);
  ShooterConfigs.MotorOutput.withPeakReverseDutyCycle(0.5);
  ShooterConfigs.MotionMagic.withMotionMagicAcceleration(100);
  ShooterConfigs.MotionMagic.withMotionMagicCruiseVelocity(150);
  ShooterConfigs.MotionMagic.withMotionMagicJerk(150);
  ShooterConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
  Shooter.getConfigurator().apply(ShooterConfigs,0.050);



  GroundIntake.setInverted(false);
  TalonFXConfiguration GroundIntakeConfigs = new TalonFXConfiguration();

  GroundIntakeConfigs.MotorOutput.withPeakForwardDutyCycle(0.5);
  GroundIntakeConfigs.MotorOutput.withPeakReverseDutyCycle(0.5);
  GroundIntakeConfigs.MotionMagic.withMotionMagicAcceleration(100);
  GroundIntakeConfigs.MotionMagic.withMotionMagicCruiseVelocity(150);
  GroundIntakeConfigs.MotionMagic.withMotionMagicJerk(150);
  GroundIntakeConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
  GroundIntake.getConfigurator().apply(GroundIntakeConfigs,0.050);



  Feeder.getConfigurator().apply(new TalonFXConfiguration());  
  TalonFXConfiguration FeederRightRollerConfigs = new TalonFXConfiguration();
  Feeder.setInverted(true);
  FeederRightRollerConfigs.MotorOutput.withPeakForwardDutyCycle(0.5);
  FeederRightRollerConfigs.MotorOutput.withPeakForwardDutyCycle(0.5);
  FeederRightRollerConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
  Feeder.getConfigurator().apply(FeederRightRollerConfigs,0.050);



  IntakeRoller.getConfigurator().apply(new TalonFXConfiguration());  
  TalonFXConfiguration IntakeRollerConfigs = new TalonFXConfiguration();
  IntakeRoller.setInverted(false);
  IntakeRollerConfigs.MotorOutput.withPeakForwardDutyCycle(0.5);
  IntakeRollerConfigs.MotorOutput.withPeakReverseDutyCycle(-0.5);
  IntakeRollerConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
  IntakeRoller.getConfigurator().apply(IntakeRollerConfigs,0.050);







  ShootercanCoder.getConfigurator().apply(new CANcoderConfiguration()); 
  CANcoderConfiguration ShooterCanCoderConfiguration = (new CANcoderConfiguration());


    reset();
// reset everything
}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putBoolean("Note inside??", sensor());
    
    // if(BeamBreakIntake.get() == true){

    //   }

    // if(BeamBreakShooter.get() == true){
    //   IntakeArm.set(TalonFXControlMode.Position,160*convert);

    //   IntakeRoller.set(TalonFXControlMode.PercentOutput,0.0);
    //   FeederLeft.set(TalonFXControlMode.PercentOutput,0.0);
    //   FeederRight.set(TalonFXControlMode.PercentOutput,0.0);

    // }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
public boolean sensor(){
  return BeamBreakShooter.get();
    
  }
  
  public void reset(){

    //  IntakeArm.set(TalonFXControlMode.Position,160*convert);
    //  reset = (ShootercanCoder.getAbsolutePosition().getValue()*convertHood);
    //  Shooter.setPosition(reset);
    //  Shooter.set(TalonFXControlMode.Position,220*convert1);

    
      // IntakeArm.set(TalonFXControlMode.Position,reset);
     }


//  public void HoodMove(double HoodAngle){
//    Shooter.setPosition(HoodAngle*convertHood);

//  }

 public void FeederMove(double Feederpercent){
    Feeder.setControl(new DutyCycleOut(Feederpercent));


 }
 public void GIntakeMove(double Gintakepercent){
  GroundIntake.setControl(new DutyCycleOut(Gintakepercent));
 }

 public void RollerMove(double Rollerpercent){
    IntakeRoller.setControl(new DutyCycleOut(Rollerpercent));

}}
  


  