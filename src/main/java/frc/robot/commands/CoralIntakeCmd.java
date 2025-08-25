package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.CoralIntake;

public class CoralIntakeCmd extends Command {
    private final CoralIntake coralIntake;
    private final Supplier<Boolean> isIntake;
    private final Supplier<Boolean> isIntake2;
    private final Supplier<Boolean> isOuttake;
    

    public CoralIntakeCmd(CoralIntake coralIntake,
            Supplier<Boolean> isIntake,
            Supplier<Boolean> isIntake2,
            Supplier<Boolean> isOuttake
            
    ) {
        this.coralIntake = coralIntake;
        this.isIntake = isIntake;
        this.isIntake2 = isIntake2;

        this.isOuttake = isOuttake;
        addRequirements(this.coralIntake);
    }

    @Override
    public void execute() {
        if (this.isIntake.get())
            this.coralIntake.execute(ControllerConstants.CORALINTAKE_SPEED);
        else if (this.isIntake2.get())
            this.coralIntake.execute(ControllerConstants.CORALINTAKE2_SPEED);
        else if (this.isOuttake.get())
            this.coralIntake.execute(-ControllerConstants.CORALOUTTAKE_SPEED);
        else
            this.coralIntake.stopIntake();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {
        this.coralIntake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}