package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Headless robot")
public class HeadlessRobot extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();

    boolean arcadeMode = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        RobotCommands commands = new RobotCommands();
        commands.init(robot);
        commands.initGyro();


        while(robot.gyro().isCalibrating()){
            telemetry.addLine("Calibrating Dont Move");
            telemetry.update();
        }
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            robot.gyro().calibrate();
        }
        if (gamepad1.a) {
            arcadeMode = !arcadeMode;
        }
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Heading (reset: x)", robot.gyro().getHeading());
        telemetry.update();

        final double x = Math.pow(gamepad1.left_stick_x, 3.0);
        final double y = Math.pow(gamepad1.left_stick_y, 3.0);

        final double rotation = Math.pow(gamepad1.right_stick_x, 3.0);
        final double direction = Math.atan2(x, y) + (arcadeMode ? robot.gyro().getHeading(): 0.0);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double lf = -speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rf = -speed * Math.sin(direction + Math.PI / 4.0) - rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double rr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;

        robot.frontLeft().setPower(lf);
        robot.frontRight().setPower(rf);
        robot.backLeft().setPower(lr);
        robot.backRight().setPower(rr);

    }
}
