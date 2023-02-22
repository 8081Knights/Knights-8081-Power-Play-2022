package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.teamcode.ConeOrientate;
import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;
//import org.firstinspires.ftc.teamcode.FusionFramework.GyroSensor;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Camera Orientation Test")

public class CamOrientTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands = new RobotCommands();
    ConeOrientate detector = new ConeOrientate();
    OpenCvCamera camera;

    boolean aDown = false;
    boolean b = false;
    boolean success = false;

    double pxDeg = 15;
    double coneDegree = 0;
    double coneTolerance = 4;
    double center = 395;




    @Override
    public void init() {
        robot.init(hardwareMap);
        camera = robot.getFrontWebCam();
        commands.init(robot);
        robot.clawElbow().setPosition(1);
        robot.clawWrist().setPosition(0);
        robot.clawGrab().setPosition(0.55);


        robot.gyro().initialize();
        robot.gyro().calibrate();

        while(robot.gyro().isCalibrating()){

            telemetry.addLine("Gyro is calibrating");
            telemetry.update();

        }

        telemetry.clear();
        telemetry.addLine("ur cringe");
        telemetry.update();




        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });



    }

    @Override
    public void loop() {

        if(gamepad1.a){
//            detector.sortCont();
//            telemetry.addData("Silly cone position: ", detector.conePos());
//            telemetry.update();
            aDown = true;
        }


        if(aDown && !gamepad1.a){
            boolean busy = detector.isBusy();
            if(busy) {
                telemetry.addLine("busy!");
                telemetry.update();
            }
            else{
                detector.sortCont(telemetry, detector.getContourList());
            }


            aDown = false;
//
            coneDegree = (detector.conePos().x-center)/pxDeg;


        }


        if(gamepad1.b){
            b = true;
        }
        if(!gamepad1.b && b){
            commands.turnGyro(coneDegree, 500);
            b = false;
        }

        telemetry.addData("Silly cone angle: ", coneDegree);
        telemetry.addData("Silly cone position: ", detector.conePos());
        telemetry.update();




    }
}
