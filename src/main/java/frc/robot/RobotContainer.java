package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralIntakeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.joystick.Controller;

public class RobotContainer {

    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final Driver driver = new Driver();
    private final Vision vision = new Vision();
    private final CoralIntake coralIntake = new CoralIntake();


    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureButtonBindings();
        this.registerCommands();
        this.swerve.setDefaultCommand(new SwerveJoystickCmd(
                this.swerve,
                this.driver::getXLimiter,
                this.driver::getYLimiter,
                this.driver::getTurningLimiter,
                this.driver::getLeftBumperButton));

        this.coralIntake.setDefaultCommand(new CoralIntakeCmd(
                this.coralIntake,
                this.driver::isIntake,
                this.driver::isIntake2,
                this.driver::isOuttake));
        
       

            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Mod", autoChooser);
    }
    private void registerCommands() {
        NamedCommands.registerCommand("releaseCoral", this.coralIntake.releaseCoral());
        NamedCommands.registerCommand("suckCoral", this.coralIntake.suckCoral());
    }
 

    private void configureButtonBindings() {
        new Trigger(this.driver::getRightBumperButton)
                .onTrue(new InstantCommand(() -> swerve.zeroHeading()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    
}
