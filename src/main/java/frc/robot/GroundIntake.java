// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class GroundIntake extends Command {
  IntakeSubsystem intake;
    /** Creates a new IntakeSequence. 
     * @param IntakeSubsystem */
    public GroundIntake(IntakeSubsystem intake) {
      this.intake = intake;
      // Use addRequirements() here to declare subsystem dependencies.
    }
    public boolean check;

  /** Creates a new Intake. */
  public GroundIntake() {
}
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;}
 
    


  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.FeederMove(0.5);
    intake.GIntakeMove(0.5);
    intake.RollerMove(0.5);
    // intake.HoodMove(100);
    if (intake.sensor() == true){
      intake.FeederMove(0);
      intake.GIntakeMove(0);
      intake.RollerMove(0);
      // intake.HoodMove(100);
      check = true;

    } 

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(check = true){
      return true;
    }
    return false;

  }
}