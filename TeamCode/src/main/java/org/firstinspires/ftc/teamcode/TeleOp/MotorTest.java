package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "Motor Test")
public class MotorTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            robot.frontRight().setPower(0.8);
        }

        else if(gamepad1.b){
            robot.frontLeft().setPower(0.8);
        }
        else if(gamepad1.x){
            robot.backRight().setPower(0.8);
        }
        else if(gamepad1.y){
            robot.backLeft().setPower(0.8);
        }

        else{
            robot.frontRight().setPower(0);
            robot.frontLeft().setPower(0);
            robot.backRight().setPower(0);
            robot.backLeft().setPower(0);

        }

    }
}
