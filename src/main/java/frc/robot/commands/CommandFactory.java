package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pincer;
import frc.robot.subsystems.keypad;


public class CommandFactory{
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
              pincer.stopIntake();
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
              pincer.stopIntake();
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
              pincer.stopIntake();
            });
        }
    public Command scorelL4(boolean facingDownwards){
        // Added transition to avoid ramming into elevator top

        if (facingDownwards){
            return elevator.goToL(Constants.reef.reefLs.lL4)
                .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                    Constants.reef.reefToState.get(
                        Constants.reef.reefLs.lL4
                    )
                )))
                .andThen(pincer.exhaust())
                .andThen(pincer.holdState()).until(() -> !pincer.hasCoral())
                .andThen(pincer.stopIntake());
        }

        return arm.goStraightOn()
            .andThen(elevator.goToL(Constants.reef.reefLs.lL4))
            .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                Constants.reef.reefToState.get(
                    Constants.reef.reefLs.lL4
                )
            )))
            .andThen(pincer.exhaust())
                .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
            .finallyDo((interrupted) ->{
                    pincer.stopIntake();
                });
    }

    public Command goTolL4(){
        return elevator.goToL(Constants.reef.reefLs.lL4)
                .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                    Constants.reef.reefToState.get(
                        Constants.reef.reefLs.lL4
                    )
                )));
    }

    public Command exhaustCoral(){
        return pincer.exhaust()
                .andThen(pincer.holdState()).until(() -> !pincer.hasCoral())
                .andThen(pincer.stopIntake());
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
                    pincer.stopIntake();
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
                    pincer.stopIntake();
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
                    pincer.stopIntake();
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
                    pincer.stopIntake();
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
    public Command stow(boolean hasCoral, boolean hasAlgae, boolean isNearTop, boolean IsLow){
        Command output;
        if(hasCoral && !IsLow){ // coral not low
            output = pincer.stopIntake()
            .andThen(new ParallelCommandGroup(arm.coralStow(),
            elevator.coralStow())
            ).andThen(pincer.pincerFunnel());
        }
        else if(hasCoral && IsLow){ // coral low
            output = pincer.stopIntake()
            .andThen(elevator.coralStow())
            .andThen(arm.coralStow())
            .andThen(pincer.pincerFunnel());
        }
        else if(hasAlgae){ // no coral -- algae or low
            output = pincer.stopIntake()
            .andThen(new ParallelCommandGroup(arm.algaeStow(),
            elevator.algaeStow()))
            .andThen(pincer.pincerAlgaeHold());
        }
        else if(isNearTop) {
            output =  pincer.stopIntake()
                .andThen(new ParallelCommandGroup(arm.goStraightOn()
                , elevator.emptyStow()))
                .andThen(arm.emptyStow())
                .andThen(pincer.pincerFunnel());
        }
        else{
            output = pincer.stopIntake()
            .andThen(new ParallelCommandGroup(arm.emptyStow()
            , elevator.emptyStow()))
            .andThen(pincer.pincerFunnel());
            }

        return output;
    }
    /*move claw, pivot, elevator to intake */
    public Command feed(){
        return //pincer.pincerFunnel()
        new ParallelCommandGroup(elevator.goToFeed(),
         arm.pivotToFeed())
         .andThen(pincer.pincerFunnel())
         .andThen(pincer.intake())
         .andThen(pincer.holdState().until(()->pincer.hasCoral()))
         .andThen(pincer.stopIntake());
    }

    public Command reefAlgaeHigh(){
            return //pincer.pincerAlgae()
            elevator.reefAlgaeHigh()
            .andThen(arm.reefAlgaeHigh())
            //.andThen(pincer.reefAlgae())
            .andThen(pincer.intake())
            //.andThen(new WaitUntilCommand(()->pincer.hasAlgae()))
            .andThen(pincer.pincerAlgaeHold());
            //.andThen(Commands.waitUntil(() -> pincer.hasAlgae()))
            //.andThen(arm.pivotToParallel());
            //.until(() -> pincer.hasAlgae())
            //.finallyDo((interrupted) -> pincer.stopIntake());
        }
        
    public Command reefAlgaeLow(){
        
            return //pincer.pincerAlgae()
            elevator.reefAlgaeLow()
            .andThen(arm.reefAlgaeLow())
            //andThen(pincer.reefAlgae())
            .andThen(pincer.intake())
            //.andThen(new WaitUntilCommand(()->pincer.hasAlgae()))
            .andThen(pincer.pincerAlgaeHold());
            //.andThen(Commands.waitUntil(() -> pincer.hasAlgae()))
            //.andThen(arm.pivotToParallel());
            //.until(() -> pincer.hasAlgae())
            //.finallyDo((interrupted) -> pincer.stopIntake());
    }

/*score net net */
    public Command net(){
        return new ParallelCommandGroup(elevator.goToNet(),
        arm.goToNet())
        .andThen(pincer.exhaust())
        .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->{pincer.stopIntake(); pincer.pincerFunnel();});
    }
    /*score processor */
    public Command processor(){
        /* return elevator.goToProcessor()
        .andThen(arm.goToProcessor())
        
        .andThen(pincer.exhaust())
        .finallyDo((interrupted) ->{
            pincer.stopIntake();
          }); */

        return new ParallelCommandGroup(elevator.goToProcessor(),
        arm.goToProcessor())
        .andThen(pincer.exhaust())
        .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->
              {pincer.stopIntake(); pincer.pincerFunnel();});  
    }
    public Command groundAlgae(){
        return new ParallelCommandGroup(elevator.goToGroundAlgae(),
        arm.goToGroundAlgae())
        //.andThen(pincer.intake())
        .andThen(pincer.pincerAlgaeHold());
        //.finallyDo((interrupted) ->{
        //    pincer.stopIntake().schedule();
        //  });
    }

    
    


}