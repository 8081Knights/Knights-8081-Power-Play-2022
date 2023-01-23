package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@Autonomous(name="Turn Gyro test")
public class GyroTest extends LinearOpMode {

    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        commands = new RobotCommands();
        commands.init(robot);

        commands.initGyro();

        while(robot.gyro().isCalibrating()){
            telemetry.addLine("Calibrating Gyro Don't Move!!");

        }
        telemetry.update();

        waitForStart();

        commands.turnGyro(30, 100);

    }
}
