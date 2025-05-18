package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constArm;

public class SetArmPosCommand extends Command {

    double shoulderAngle;
    double elbowAngle;
    double wristAngle;
    double targetX;
    double targetY;

    public SetArmPosCommand(double x, double y) {
        targetX = x - constArm.L0;
        targetY = y;
        double distanceToTarget = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));
        if (Math.abs( constArm.L1 - constArm.L2 ) > distanceToTarget || distanceToTarget > constArm.L1 + constArm.L2) {
            // cannot reach target
        }
        
    }

}
