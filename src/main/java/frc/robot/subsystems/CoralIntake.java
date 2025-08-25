
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

public class CoralIntake extends SubsystemBase {
    private final SparkFlex coralMotor;

    public CoralIntake() {
        this.coralMotor = new SparkFlex(10, MotorType.kBrushless);
    }

    public void execute(double speed) {
        this.coralMotor.set(-speed); }

    public Command releaseCoral() {
        return Commands.runOnce(() -> this.execute(-ControllerConstants.CORALOUTTAKE_SPEED), this);
    }
    
    public Command suckCoral() {
        return Commands.runOnce(() -> this.execute(ControllerConstants.CORALINTAKE_SPEED), this);
    }

    public void stopIntake() {
        this.coralMotor.stopMotor();
    }
}

