package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name="scrimmage teleop")
public class scrimmageTeleop extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    int maxSpeed = 2000;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.frontRight().setVelocity((int)(v1*maxSpeed));
        robot.frontLeft( ).setVelocity((int)(v2*maxSpeed));
        robot.backRight( ).setVelocity((int)(v3*maxSpeed));
        robot.backLeft(  ).setVelocity((int)(v4*maxSpeed));

        if(gamepad2.a){
            robot.clawChange();
        }
        if(gamepad2.right_trigger > 0.1){
            robot.armUp(1000);
        }
    }
}
