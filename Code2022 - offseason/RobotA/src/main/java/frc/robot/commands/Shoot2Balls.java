// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.LimelightBase;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot2Balls extends SequentialCommandGroup {
  private DriveBase driveBase;
  private IntakeBase intakeBase;
  private StorageSubsystem storageSubsystem;
  private ShooterBase shooterBase;
  private LimelightBase limelightBase;
  public Shoot2Balls(DriveBase driveBase, IntakeBase intakeBase, StorageSubsystem storageSubsystem, ShooterBase shooterBase, LimelightBase limelightBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.driveBase = driveBase;
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;
    this.shooterBase = shooterBase;
    this.limelightBase = limelightBase;

    
    
    
    addCommands(


      new InstantCommand(()->driveBase.setVelocity(-1,-1), driveBase),  
      new WaitCommand(1),
      new InstantCommand(()->driveBase.setVelocity(0,0), driveBase),

      new ParallelRaceGroup(new AimToHub(limelightBase, driveBase), new WaitCommand(2)),

      new ParallelRaceGroup(new SetCalcShooterSpeed(shooterBase, storageSubsystem, limelightBase), new WaitCommand(1.3)),
      new ParallelRaceGroup(new MoveStorageAuto(storageSubsystem), new WaitCommand(1.7)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase)


      
    );
  }
}
