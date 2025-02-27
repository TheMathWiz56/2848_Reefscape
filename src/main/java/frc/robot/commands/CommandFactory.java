package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pincer;
import frc.robot.subsystems.keypad;


public class CommandFactory {
    private final CommandSwerveDrivetrain drive;
    private final Elevator elevator;
    private final Arm arm;
    private final Pincer pincer;
    private final Lights lights;

    public CommandFactory(CommandSwerveDrivetrain drive, Elevator elevator, Arm arm, Pincer pincer, Lights lights){
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
        this.pincer = pincer;
        this.lights = lights;
    }
/*Moves only elevator, pivot and intake to score on reef */
    // public Command scoreL(Supplier<Constants.reef.reefLs> L,Supplier<Integer> reef){
    //     return elevator.goToL(L.get(),reef.get())
    //         // .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
    //         //     Constants.reef.reefToState.get(L.get())
    //         // )))
    //         .andThen(pincer.exhaust())
    //          .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
    //         .finallyDo((interrupted) ->{
    //               pincer.stopIntake().schedule();
    //             });
    // }
    // public Command scoreL(Constants.reef.reefLs L,int reef){
    //     return elevator.goToL(L,reef)
    //         // .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
    //         //     Constants.reef.reefToState.get(L)
    //         // )))
    //         .andThen(pincer.exhaust())
    //          .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
    //         .finallyDo((interrupted) ->{
    //               pincer.stopIntake().schedule();
    //             });
    // }

    public Command scorelL1(){
        return elevator.goToL(Constants.reef.reefLs.lL1)
        .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
            Constants.reef.reefToState.get(
                Constants.reef.reefLs.lL1
            )
        )))
        .andThen(pincer.exhaust())
         .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->{
              pincer.stopIntake().schedule();
            });
    }
    public Command scorelL2(){
        return elevator.goToL(Constants.reef.reefLs.lL2)
        .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
            Constants.reef.reefToState.get(
                Constants.reef.reefLs.lL2
            )
        )))
        .andThen(pincer.exhaust())
         .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->{
              pincer.stopIntake().schedule();
            });
    }
    public Command scorelL3(){
        return elevator.goToL(Constants.reef.reefLs.lL3)
        .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
            Constants.reef.reefToState.get(
                Constants.reef.reefLs.lL3
            )
        )))
        .andThen(pincer.exhaust())
         .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->{
              pincer.stopIntake().schedule();
            });
        }
        public Command scorelL4(){
            return elevator.goToL(Constants.reef.reefLs.lL4)
            .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                Constants.reef.reefToState.get(
                    Constants.reef.reefLs.lL4
                )
            )))
            .andThen(pincer.exhaust())
             .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
            .finallyDo((interrupted) ->{
                  pincer.stopIntake().schedule();
                });
            }
            public Command scorerL1(){
                return elevator.goToL(Constants.reef.reefLs.rL1)
                .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                    Constants.reef.reefToState.get(
                        Constants.reef.reefLs.rL1
                    )
                )))
                .andThen(pincer.exhaust())
                 .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
                .finallyDo((interrupted) ->{
                      pincer.stopIntake().schedule();
                    });
            }
            public Command scorerL2(){
                return elevator.goToL(Constants.reef.reefLs.rL2)
                .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                    Constants.reef.reefToState.get(
                        Constants.reef.reefLs.rL2
                    )
                )))
                .andThen(pincer.exhaust())
                 .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
                .finallyDo((interrupted) ->{
                      pincer.stopIntake().schedule();
                    });
            }
            public Command scorerL3(){
                return elevator.goToL(Constants.reef.reefLs.rL3)
                .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                    Constants.reef.reefToState.get(
                        Constants.reef.reefLs.rL3
                    )
                )))
                .andThen(pincer.exhaust())
                 .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
                .finallyDo((interrupted) ->{
                      pincer.stopIntake().schedule();
                    });
                }
                public Command scorerL4(){
                    return elevator.goToL(Constants.reef.reefLs.rL4)
                    .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                        Constants.reef.reefToState.get(
                            Constants.reef.reefLs.rL4
                        )
                    )))
                    .andThen(pincer.exhaust())
                     .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
                    .finallyDo((interrupted) ->{
                          pincer.stopIntake().schedule();
                        });
                    }
    // public Command scoreL(){
    //     return new WaitUntilCommand(()->elevator.isDone())
    //         .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
    //             Constants.reef.reefToState.get(L.get())
    //         )))
    //         .andThen(pincer.exhaust())
    //          .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
    //         .finallyDo((interrupted) ->{
    //               pincer.stopIntake().schedule();
    //             });
    // }
/*stows. Uses sensor to determine which stow */
    public Command stow(){
        if(pincer.hasCoral()){
            return new InstantCommand(()->pincer.stopIntake(),pincer)
            .andThen(arm.coralStow())
            .andThen(elevator.coralStow()
            );
        }
        if(pincer.hasAlgae()){
            return new InstantCommand(()->pincer.stopIntake(),pincer)
            .andThen(arm.algaeStow())
            .andThen(elevator.algaeStow());
        }
        return new InstantCommand(()->pincer.stopIntake(),pincer)
        .andThen(arm.emptyStow())
        .andThen(elevator.emptyStow());

    }
    /*move claw, pivot, elevator to intake */
    public Command feed(){
        return //pincer.pincerFunnel()
        elevator.goToFeed()
         .andThen(arm.pivotToFeed())
         .andThen(pincer.intake())
         .andThen(pincer.holdState())
         .until(()->pincer.hasCoral())
         .finallyDo(()-> pincer.stopIntake().schedule());
    }

    public Command reefAlgaeHigh(){
            return elevator.reefAlgaeHigh()
            .andThen(arm.reefAlgaeHigh())
            .andThen(pincer.reefAlgae())
            .andThen(pincer.intake());
        }
        
    public Command reefAlgaeLow(){
        
            return elevator.reefAlgaeLow()
            .andThen(arm.reefAlgaeLow())
            .andThen(pincer.reefAlgae())
            .andThen(pincer.intake());
        
    }
/*score net net */
    public Command net(){
        return elevator.goToNet()
        .andThen(arm.goToNet())
        .andThen(pincer.exhaust())
        .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->{
              pincer.stopIntake().schedule();
            });
    }
    /*score processor */
    public Command processor(){
        return elevator.goToProcessor()
        .andThen(arm.goToProcessor())
        .andThen(pincer.exhaust())
        .finallyDo((interrupted) ->{
            pincer.stopIntake().schedule();
          });
    }
    public Command groundAlgae(){
        return elevator.goToGroundAlgae()
        .andThen(arm.goToGroundAlgae())
        .andThen(pincer.exhaust())
        .finallyDo((interrupted) ->{
            pincer.stopIntake().schedule();
          });
    }

    
    
    



}