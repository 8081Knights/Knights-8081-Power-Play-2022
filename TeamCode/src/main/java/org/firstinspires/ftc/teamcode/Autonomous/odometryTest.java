//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.HardwareSoftware;
//
//@Autonomous(name="Odometry Test Code")
//
//public class odometryTest extends LinearOpMode {
//
//    HardwareSoftware robot = new HardwareSoftware();
//
//    public void odoMove(int distanceCm) {
//        //method for moving robot using odometry. set parameter to distance in centimeters and the robot should move that far
//
//        //initializing variables for method
//        int targetEncoderValue = distanceCm*522; //522 is the number of ticks the encoder turns per centimeter of distance it moves
//
//        int distanceToTarget = targetEncoderValue - robot.encoder().getCurrentPosition();
//
//        int distanceMagnitude = Math.abs(distanceToTarget);
//
//        double motorPower = 0;
//
//
//        int tolerance = 300;
//
//        int[] distanceCurve = {50000, 25000, 10000, 500}; //these 2 arrays should be same length with each value corresponding to the value in the same place at the other array.
//
//        double[] powerCurve = {0.5, 0.3, 0.15, 0.1}; // You put in a distance from target for the distance array and a motor power for the power array.
//
//
//        while (-tolerance > distanceToTarget || distanceToTarget > tolerance){ //same while loop twice so that it can wait a moment then double check that it is within the target range
//
//            while (-tolerance > distanceToTarget || distanceToTarget > tolerance) {
//
//                distanceMagnitude = Math.abs(distanceToTarget);
//
//                motorPower = 0.8;
//
//                for (int i = 0; i <= (distanceCurve.length-1); i++) {
//                    if (distanceMagnitude < distanceCurve[i]){
//                        motorPower = powerCurve[i];
//                    }
//                }
//
//                if (distanceToTarget < 0) {
//                    motorPower = -motorPower;
//                }
//
//                runDriveTrain(motorPower, motorPower);
//
//                telemetry.addData("Power:", motorPower);
//                telemetry.addData("Distance:", distanceToTarget);
//                telemetry.update();
//
//                distanceToTarget = targetEncoderValue - robot.encoder().getCurrentPosition();
//
//            }
//
//            runDriveTrain(0, 0);
//
//            sleep(500);
//
//            distanceToTarget = targetEncoderValue - robot.encoder().getCurrentPosition();
//            telemetry.addData("Power:", motorPower);
//            telemetry.addData("ticks to destination:", distanceToTarget);
//            telemetry.addData("Distance CM:", (robot.encoder().getCurrentPosition()/522));
//            telemetry.update();
//
//        }
//
//       odoReset();
//
//    }
//
//    public void runDriveTrain(double leftPower, double rightPower){
//        //sets power for all drivetrain motors
//        robot.frontLeft().setPower(-leftPower);
//        robot.backLeft().setPower(-leftPower);
//        robot.frontRight().setPower(rightPower);
//        robot.backRight().setPower(rightPower);
//    }
//
//    public void odoReset() {
//        //resets the encoder to zero
//        robot.encoder().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        robot.encoder().setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        odoReset();
//
//
//        waitForStart();
//
//
//        odoMove(122);
//
//        sleep(3000);
//    }
//}