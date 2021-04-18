package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.motors.REVHDHEXHUB_1291;
import org.firstinspires.ftc.teamcode.Odometry.RobotMovement;


import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collector;

import static org.firstinspires.ftc.teamcode.Odometry.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.Odometry.RobotMovement.getFollowPointPath;

/**
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    DcMotorEx lift;

    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(REVHDHEXHUB_1291.class);
    final double COUNTS_PER_INCH = MOTOR_CONFIG.getTicksPerRev();

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "FRWheelandLeftOdometry", rbName = "BRWheelandCenterOdometry", lfName = "FLWheelM", lbName = "BLWheelM", rightOdo="CollectorMandRightOdometry";
    String verticalLeftEncoderName = rfName, verticalRightEncoderName = rightOdo, horizontalEncoderName = rbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    BNO055IMU imu;

    public DcMotor bl;
    public DcMotor fl;
    public DcMotor br;
    public DcMotor fr;
    public DcMotor collector;
    public DcMotorEx shooter;
    public DcMotor advancingM;

    public boolean shooterToggle = false;
    public boolean triggered = false;
    //public int topHeight = 0;
    //public int bottomHeight = 1300;
    public boolean liftingUp = false;
    public boolean falling = false;
    public int MINPOSITION = 100;
    public int MAXPOSITION = 2520;
    public int collectorVal = 0;
    public final int TRIGGERFORWARD =  1480;
    public final int TRIGGERBACK = 1140;
    public final int HIGHSHOT = 985;
    public final int HIGHSHOT2 = 845;
    public final int LOWSHOT = 1800;
    public int shootHeight = HIGHSHOT;
    public final int HIGHPOS = -1210;
    public final int ELBOWUP = 1200;
    public final int ELBOWDOWN = 2200;
    boolean firstTime = true;

    public int wobbleGoalX = 0;
    public int WobbleX1 = -17;
    public int wobbleGoalY = 0;
    public int WobbleY1 = 70;
    public int WobbleX2 =10;
    public int WobbleY2 = 90;
    public int WobbleX3 =25;
    public int WobbleY3 = 130;
    public int elbowH = 1600;
    public int CLOSEDPOS = 550;
    public int OPENPOS = 2000;
    public int handH = CLOSEDPOS;
    public Servo trigger;
    public Servo shooterHeights;
    public Servo shooterHeights2;
    public Servo elbow;
    public Servo hand;
    public Servo block;
    public int size = 0;
    public String label = "quad";
    public double robotPower1 = 4.0;
    public double robotPower2 = 0.5;


    private static final String VUFORIA_KEY =
            "AQBzB4f/////AAABmVmKcSloiEn3pJ6FLFPDeTQd2Szru8mc0liicGapN60zlHIX6GeYtjuRt5GJam1rpOXf6cCvjw/M+fIbCYdwhoycNC2HJoLwpd3J5VmagW4sUqjyfoG0A76sPxE838m6fY/6gj9d121/bEn53FDE6i3lTDF8cLHttMTzVcqAPMqm9jEMxSFidjG+vvq70PN8V/Bd46qo7pFzMUrnkYZ5NgtYdddxAeI3rT9h/U1EHFqKaaRThB+RmmGL899VKPgGib+pXT0Wi6SCbjBBnXLPA4W8z4KHQ9XdIGxoar4UWm2eNP4QGBG6I7SRuIzr7xESNTtNljhx0vs0BpPcDMzDHi2FIcABtSyM4U79Wte9lUYI";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public final double cpi = MOTOR_CONFIG.getTicksPerRev();
    Orientation lastAngles = new Orientation();
    double  globalAngle, power = .30, correction, rotation;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initGyro(imu, hardwareMap);
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        //telemetry.addData("Status", "Init Complete");
        //telemetry.update();
        shooterHeights.setPosition((shootHeight-100.0)/ 2420.0);
        shooterHeights2.setPosition((shootHeight-100.0)/ 2420.0);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.0, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    label = recognition.getLabel();
                }
                size = updatedRecognitions.size();
                telemetry.update();
            }
        }


        waitForStart();


        if(firstTime)
        {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        label = recognition.getLabel();
                    }
                    size = updatedRecognitions.size();
                    telemetry.update();
                }
            }
        }

        if(size == 0)
        {
            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
            globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 20);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            Thread st = (Thread) new ShooterThread();
            st.start();

            //goToPosition(telemetry, 30, 30, 0.85, Math.PI/2, 1.0);

            telemetry.addData("Hardware map initialized", 1);
            telemetry.update();

            hand.setPosition((CLOSEDPOS - 100.0) / 2420.0);
            block.setPosition((1850-100.0)/ 2420.0);

            //Thread.sleep(50);
            elbow.setPosition((1300.0 - 100.0) / 2420.0);
            shooterHeights.setPosition((HIGHSHOT - 100.0) / 2420.0);
            shooterHeights2.setPosition((HIGHSHOT - 100.0) / 2420.0);
            shooter.setVelocity(1500);
            advancingM.setPower(-1.00);

            //rotate(85, 0.7);

            //sleep(20000);

            //turnWithGyro(left_front, left_back, right_front, right_back, imu, 90, 0.3);
            //goToPosition(telemetry,15, 15, 0.75, 0, 1);
            //sleep(1000);

            ArrayList<CurvePoint> allPoints3 = new ArrayList<>();
            allPoints3.add(new CurvePoint(WobbleX1,WobbleY1+3,0.95,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints3, Math.toRadians(Math.toRadians(0)), 3.5, st);


            ArrayList<CurvePoint> allPoints4 = new ArrayList<>();

            allPoints4.add(new CurvePoint(WobbleX1-25,WobbleY1-7,0.95,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints4, Math.toRadians(Math.toRadians(0)), 3.5, st);



            ArrayList<CurvePoint> allPoints5 = new ArrayList<>();

            allPoints5.add(new CurvePoint(WobbleX1-10,WobbleY1+15.5,0.85,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints5, Math.toRadians(Math.toRadians(0)), 3.5, st);

            Thread.sleep(450);



            ArrayList<CurvePoint> allPoints6 = new ArrayList<>();

            allPoints6.add(new CurvePoint(WobbleX1+35,WobbleY1+15.5,0.90,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints6, Math.toRadians(Math.toRadians(0)), 3.5, st);




            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(   -8,67,0.80,1.0,2, Math.toRadians(0), 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints, Math.toRadians(0), 1.0, st);



            //elbow.setPosition((ELBOWDOWN - 100.0) / 2420.0);
            liftingUp=true;
            //lift.setPower(-0.4);
            falling = false;
            st.start();

            Thread.sleep(1000);

            toggleTriggerThrice();
            Thread.sleep(200);

            liftingUp = false;
            falling = true;




            //hand.setPosition((OPENPOS - 100.0) / 2420.0);

            //Thread.sleep(500);
        /*
        left_back.setPower(0.0);
        left_front.setPower(0.0);
        right_back.setPower(0.0);
        right_front.setPower(0.0);

*/



            ArrayList<CurvePoint> allPoints8 = new ArrayList<>();
            allPoints8.add(new CurvePoint(   0, 1,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints8, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints9 = new ArrayList<>();
            allPoints9.add(new CurvePoint(   16.7, 1,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints9, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints10 = new ArrayList<>();
            allPoints10.add(new CurvePoint(   30.5, WobbleY1-3,0.80,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints10, 0f, 3.0, st);


            ArrayList<CurvePoint> allPoints11 = new ArrayList<>();
            allPoints11.add(new CurvePoint(   30, WobbleY1-18,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints11, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints12 = new ArrayList<>();
            allPoints12.add(new CurvePoint(   0, WobbleY1-25,0.80,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints12, 0f, 2.5, st);


            ArrayList<CurvePoint> allPoints7 = new ArrayList<>();
            allPoints7.add(new CurvePoint(   -10, 88,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints7, 0f, 2.5, st);


            //Thread.sleep(100);
        }
        else if(label.equalsIgnoreCase("quad"))
        {
            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
            globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 40);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            Thread st = (Thread) new ShooterThread();
            st.start();

            //goToPosition(telemetry, 30, 30, 0.85, Math.PI/2, 1.0);

            telemetry.addData("Hardware map initialized", 1);
            telemetry.update();

            block.setPosition((1850-100.0)/ 2420.0);

            hand.setPosition((CLOSEDPOS - 100.0) / 2420.0);
            //Thread.sleep(50);
            elbow.setPosition((1300.0 - 100.0) / 2420.0);
            shooterHeights.setPosition((HIGHSHOT - 100.0) / 2420.0);
            shooterHeights2.setPosition((HIGHSHOT - 100.0) / 2420.0);

            shooter.setVelocity(1500);
            advancingM.setPower(-1.00);

            liftingUp = true;

            falling = false;

            //rotate(85, 0.7);

            //sleep(20000);

            //turnWithGyro(left_front, left_back, right_front, right_back, imu, 90, 0.3);
            //goToPosition(telemetry,15, 15, 0.75, 0, 1);
            //sleep(1000);



            ArrayList<CurvePoint> allPoints09 = new ArrayList<>();
            allPoints09.add(new CurvePoint(-3,3.65,1.0,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints09, Math.toRadians(Math.toRadians(0)), 3.0, st);

            ArrayList<CurvePoint> allPoints0 = new ArrayList<>();
            allPoints0.add(new CurvePoint(17,0,0.98,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints0, Math.toRadians(Math.toRadians(0)), 3.0, st);


            //ArrayList<CurvePoint> allPoints00 = new ArrayList<>();
            //allPoints00.add(new CurvePoint(WobbleX1+23, WobbleY3,0.95,1.0,2, Math.toRadians(0), 0.5));

            //followCurve(COUNTS_PER_INCH, telemetry, allPoints00, Math.toRadians(Math.toRadians(0)), 3.0, st);


            //ArrayList<CurvePoint> allPoints01 = new ArrayList<>();
            //allPoints01.add(new CurvePoint(WobbleX1-10, WobbleY3-15,0.95,1.0,2, Math.toRadians(0), 0.5));

            //followCurve(COUNTS_PER_INCH, telemetry, allPoints01, Math.toRadians(Math.toRadians(0)), 3.0, st);

           // ArrayList<CurvePoint> allPoints02 = new ArrayList<>();
            //allPoints02.add(new CurvePoint(WobbleX1-10, WobbleY3+10,0.95,1.0,2, Math.toRadians(0), 0.5));

 //           followCurve(COUNTS_PER_INCH, telemetry, allPoints02, Math.toRadians(Math.toRadians(0)), 3.0, st);



            Thread.sleep(250);


            ArrayList<CurvePoint> allPoints3 = new ArrayList<>();
            allPoints3.add(new CurvePoint(WobbleX1+48.5,WobbleY3+8,5.0,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints3, Math.toRadians(Math.toRadians(0)), 3.6, st);

            /*
            ArrayList<CurvePoint> allPoints03 = new ArrayList<>();
            allPoints03.add(new CurvePoint(WobbleX1+40,WobbleY3+10,1.0,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints03, Math.toRadians(Math.toRadians(0)), 3.0, st);


             */



            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(   -5,70,0.93,1.0,2, Math.toRadians(0), 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints, Math.toRadians(0), 2.3, st);



            //elbow.setPosition((ELBOWDOWN - 100.0) / 2420.0);
            /*
            liftingUp=true;
            //lift.setPower(-0.4);
            falling = false;
            st.start();

             */

            Thread.sleep(200);

            toggleTriggerThrice();
            Thread.sleep(300);

            liftingUp = false;
            falling = true;

            block.setPosition((279-100.0)/ 2420.0);






            //hand.setPosition((OPENPOS - 100.0) / 2420.0);

            //Thread.sleep(500);
        /*
        left_back.setPower(0.0);
        left_front.setPower(0.0);
        right_back.setPower(0.0);
        right_front.setPower(0.0);

*/

            ArrayList<CurvePoint> allPoints08 = new ArrayList<>();
            allPoints08.add(new CurvePoint(   -28, 60,1.0,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints08, 0f, 3.0, st);


            ArrayList<CurvePoint> allPoints001 = new ArrayList<>();
            allPoints001.add(new CurvePoint(   -12, 35,1.0,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints001, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints005 = new ArrayList<>();
            allPoints005.add(new CurvePoint(   3, 31,1.0,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints005, 0f, 3.0, st);

            collector.setPower(-1.00);
            liftingUp = false;
            falling = true;
            st.start();


            ArrayList<CurvePoint> allPoints006 = new ArrayList<>();
            allPoints006.add(new CurvePoint(   3, 41,0.78,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints006, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints007 = new ArrayList<>();
            allPoints007.add(new CurvePoint(   3, 36,0.78,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints007, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints008 = new ArrayList<>();
            allPoints008.add(new CurvePoint(   3, 46,0.78,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints008, 0f, 3.0, st);

            ArrayList<CurvePoint> allPointsp1 = new ArrayList<>();
            allPointsp1.add(new CurvePoint(   3, 38,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPointsp1, 0f, 3.0, st);

            ArrayList<CurvePoint> allPointsp2 = new ArrayList<>();
            allPointsp2.add(new CurvePoint(   3, 46,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPointsp2, 0f, 3.0, st);

            shootHeight = HIGHSHOT2;

            shooterHeights.setPosition((HIGHSHOT2-100.0)/ 2420.0);
            shooterHeights2.setPosition((HIGHSHOT2-100.0)/ 2420.0);


            Thread.sleep(800);

            //elbow.setPosition((ELBOWDOWN - 100.0) / 2420.0);
            liftingUp=true;
            //lift.setPower(-0.4);
            falling = false;

            collector.setPower(0.20);


            st.start();
            Thread.sleep(800);
            toggleTriggerTwice();
            Thread.sleep(300);

            liftingUp = false;
            falling = true;

            Thread.sleep(1000);

            collector.setPower(-1.00);



            ArrayList<CurvePoint> allPoints010 = new ArrayList<>();
            allPoints010.add(new CurvePoint(   3, 51,0.92,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints010, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints0009 = new ArrayList<>();
            allPoints0009.add(new CurvePoint(   3, 46,0.92,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints0009, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints0099 = new ArrayList<>();
            allPoints0099.add(new CurvePoint(   3, 60,0.92,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints0099, 0f, 3.0, st);



            block.setPosition((1850-100.0)/ 2420.0);



            Thread.sleep(1000);

            liftingUp=true;
            //lift.setPower(-0.4);
            falling = false;


            shootHeight = HIGHSHOT2;

            shooterHeights.setPosition((HIGHSHOT2-100.0)/ 2420.0);
            shooterHeights2.setPosition((HIGHSHOT2-100.0)/ 2420.0);

            st.start();

            Thread.sleep(800);
            toggleTriggerTwice();
            Thread.sleep(300);

            liftingUp = false;
            falling = true;

            ArrayList<CurvePoint> allPoints0008 = new ArrayList<>();
            allPoints0008.add(new CurvePoint(   -30, 30,2.0,1.5,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints0008, 0f, 4.5, st);



            ArrayList<CurvePoint> allPoints8 = new ArrayList<>();
            allPoints8.add(new CurvePoint(   -20, 0,2.0,1.5,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints8, 0f, 4.5, st);

            collector.setPower(0.00);

            ArrayList<CurvePoint> allPoints9 = new ArrayList<>();
            allPoints9.add(new CurvePoint(   -3, 0,1.0,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints9, 0f, 3.0, st);

            Thread.sleep(100);

            //ArrayList<CurvePoint> allPoints10 = new ArrayList<>();
            //allPoints10.add(new CurvePoint(   18.5, 0,0.95,1.0,2, 0, 0.5));
            //followCurve(COUNTS_PER_INCH, telemetry, allPoints10, 0f, 3.0, st);

            //Thread.sleep(100);


            /*
            ArrayList<CurvePoint> allPoints003 = new ArrayList<>();
            allPoints003.add(new CurvePoint(   WobbleX3+1, 90,1.0,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints003, 0f, 3.0, st);



            ArrayList<CurvePoint> allPoints011 = new ArrayList<>();
            allPoints011.add(new CurvePoint(   WobbleX3+4,50,1.00,3.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints011, 0f, 4.0, st);


             */

            ArrayList<CurvePoint> allPoints11 = new ArrayList<>();
            allPoints11.add(new CurvePoint(   WobbleX3+13.5,WobbleY3,1.5,3.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints11, 0f, 3.0, st);


            ArrayList<CurvePoint> allPoints7 = new ArrayList<>();
            allPoints7.add(new CurvePoint(   -10, 94,3.5,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints7, 0f, 3.0, st);



            //Thread.sleep(100);
        }
        else if(label.equalsIgnoreCase("single"))
        {
            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
            block.setPosition((1850-100.0)/ 2420.0);

            globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 50);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            Thread st = (Thread) new ShooterThread();
            st.start();

            //goToPosition(telemetry, 30, 30, 0.85, Math.PI/2, 1.0);

            telemetry.addData("Hardware map initialized", 1);
            telemetry.update();

            hand.setPosition((CLOSEDPOS - 100.0) / 2420.0);
            //Thread.sleep(50);
            elbow.setPosition((1300.0 - 100.0) / 2420.0);
            shooterHeights.setPosition((HIGHSHOT - 100.0) / 2420.0);
            shooterHeights2.setPosition((HIGHSHOT - 100.0) / 2420.0);

            shooter.setVelocity(1500);
            advancingM.setPower(-1.00);

            liftingUp = true;

            falling = false;


            //rotate(85, 0.7);

            //sleep(20000);

            //turnWithGyro(left_front, left_back, right_front, right_back, imu, 90, 0.3);
            //goToPosition(telemetry,15, 15, 0.75, 0, 1);
            //sleep(1000);


            ArrayList<CurvePoint> allPoints0 = new ArrayList<>();
            allPoints0.add(new CurvePoint(-25,30,0.95,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints0, Math.toRadians(Math.toRadians(0)), 3.0, st);




            ArrayList<CurvePoint> allPoints3 = new ArrayList<>();
            allPoints3.add(new CurvePoint(WobbleX1+10, WobbleY1+18.6,0.95,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints3, Math.toRadians(Math.toRadians(0)), 3.0, st);

            ArrayList<CurvePoint> allPoints003 = new ArrayList<>();
            allPoints003.add(new CurvePoint(WobbleX1+16, WobbleY1+30,0.90,1.0,2, Math.toRadians(0), 0.5));

            followCurve(COUNTS_PER_INCH, telemetry, allPoints003, Math.toRadians(Math.toRadians(0)), 3.0, st);



            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(   -8,65,0.80,1.0,2, Math.toRadians(0), 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints, Math.toRadians(0), 1.5, st);

            //elbow.setPosition((ELBOWDOWN - 100.0) / 2420.0);
            liftingUp=true;
            //lift.setPower(-0.4);
            falling = false;
            st.start();

            Thread.sleep(1000);

            toggleTriggerThrice();
            Thread.sleep(300);

            liftingUp = false;
            falling = true;

            ArrayList<CurvePoint> allPoints0001 = new ArrayList<>();
            allPoints0001.add(new CurvePoint(   -25, 60,0.95,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints0001, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints001 = new ArrayList<>();
            allPoints001.add(new CurvePoint(   -12, 30,0.95,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints001, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints005 = new ArrayList<>();
            allPoints005.add(new CurvePoint(   0, 14,0.95,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints005, 0f, 3.0, st);

            collector.setPower(-1.00);

            ArrayList<CurvePoint> allPoints006 = new ArrayList<>();
            allPoints006.add(new CurvePoint(   0, 46,0.95,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints006, 0f, 3.0, st);


            //hand.setPosition((OPENPOS - 100.0) / 2420.0);

            ArrayList<CurvePoint> allPoints08 = new ArrayList<>();
            allPoints08.add(new CurvePoint(   -15, 20,0.95,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints08, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints8 = new ArrayList<>();
            allPoints8.add(new CurvePoint(   0, 3,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints8, 0f, 3.0, st);

            ArrayList<CurvePoint> allPoints9 = new ArrayList<>();
            allPoints9.add(new CurvePoint(   15, 3,0.80,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints9, 0f, 3.0, st);

            collector.setPower(0.00);


            ArrayList<CurvePoint> allPoints10 = new ArrayList<>();
            allPoints10.add(new CurvePoint(   20, 3,0.80,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints10, 0f, 3.0, st);
            Thread.sleep(300);

            liftingUp = true;
            falling = false;


            ArrayList<CurvePoint> allPoints11 = new ArrayList<>();
            allPoints11.add(new CurvePoint(   WobbleX1+18.5, WobbleY1+30,0.85,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints11, 0f, 3.0, st);




            //
            //

            //Thread.sleep(500);
        /*
        left_back.setPower(0.0);
        left_front.setPower(0.0);
        right_back.setPower(0.0);
        right_front.setPower(0.0);

*/

            ArrayList<CurvePoint> allPoints009 = new ArrayList<>();
            allPoints009.add(new CurvePoint(   -8,67,0.85,1.0,2, Math.toRadians(0), 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints009, Math.toRadians(0), 1.5, st);

            //elbow.setPosition((ELBOWDOWN - 100.0) / 2420.0);
            liftingUp=true;
            //lift.setPower(-0.4);
            falling = false;
            st.start();

            Thread.sleep(1000);

            toggleTrigger();
            Thread.sleep(300);

            ArrayList<CurvePoint> allPoints7 = new ArrayList<>();
            allPoints7.add(new CurvePoint(   -10, 88,0.95,1.0,2, 0, 0.5));
            followCurve(COUNTS_PER_INCH, telemetry, allPoints7, 0f, 3.0, st);

            liftingUp = false;
            falling = true;
            //Thread.sleep(100);
        }











        //goToPosition(telemetry,WobbleX, WobbleY, 1.0, -Math.PI/2, 1, st);

        //alloints.add(new CurvePoint(   50,20,1.0,1.0,50, Math.toRadians(0), 1.0));

        while(opModeIsActive()){
            //goToPosition(telemetry,15, 15, 0.75, 0, 1);

            /*
            ArrayList<CurvePoint> allPoints2 = new ArrayList<>();
            allPoints2.add(new CurvePoint(0,0,1.0,1.0,5, Math.toRadians(0), 1.0));
            allPoints2.add(new CurvePoint(50,50,1.0,1.0,5, Math.toRadians(0), 1.0));
            allPoints2.add(new CurvePoint(60,50,1.0,1.0,5, Math.toRadians(0), 1.0));
            allPoints2.add(new CurvePoint(40,20,1.0,1.0,5, Math.toRadians(0), 1.0));
            allPoints2.add(new CurvePoint(18,0,1.0,1.0,5, Math.toRadians(0), 1.0));
            */

            //goToPosition(telemetry,-30, 25, 1.0, 0, 1);
            //followCurve(COUNTS_PER_INCH, telemetry, allPoints2, Math.toRadians(0), 2);

            telemetry.addData("Moving On to next path", 2);
            telemetry.addData("Value of liftingUp", liftingUp);
            telemetry.addData("Lift position", lift.getCurrentPosition());

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
            telemetry.update();
            //Display Global (x, y, theta) coordinates
            /*
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            */

            //telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void toggleTriggerTwice() {
        triggered = true;
        trigger.setPosition((TRIGGERFORWARD-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        triggered = false;
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        triggered = true;
        trigger.setPosition((TRIGGERFORWARD-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        triggered = false;
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);

    }

    public void followAutonomousPath()
    {
        int wobbleGoal;
        //use vuphoria to determine which wobble goal you have
        wobbleGoal = 3;
        int [][] goalPositions = {
                {15, 85},
                {27, 85}
        };
        switch (wobbleGoal) {
            case 1:

        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    public void goToPosition(Telemetry tele, double targetXPos, double targetYPos, double robotPower, double desiredRobotOrientation, double allowableDistanceError)
    {
        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate()/ COUNTS_PER_INCH;
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate()/ COUNTS_PER_INCH;
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        while(opModeIsActive() && distance > allowableDistanceError) {
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
            tele.addData("Distance to target is ", distance);
            //tele.update();

            distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate()/ COUNTS_PER_INCH;
            distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate()/ COUNTS_PER_INCH;
            tele.addData("distance to x", distanceToXTarget);
            tele.addData("distance to y", distanceToYTarget);
            double robotMovementAngle = Math.atan2(distanceToYTarget, distanceToXTarget);
            tele.addData("angle is", robotMovementAngle);
            //double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            //double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            //tele.addData("x power is ", robot_movement_x_component);
            //tele.addData("y power is ", robot_movement_y_component);
            double pivotCorrection = Range.clip(desiredRobotOrientation- Math.toRadians(globalPositionUpdate.returnOrientation()), -1, 1);
            tele.addData("pivot correction is ", pivotCorrection);
            //use x, y, and pivot correction to set power to mecanum wheels on robot
            //Try using these to figure it out - make rightX the pivto and r the hypotenus of x and y

            double vlb = robotPower * Math.sin(robotMovementAngle - Math.PI/4) + (pivotCorrection/2);
            double vrf = robotPower * Math.sin(robotMovementAngle - Math.PI/4) - (pivotCorrection/2);
            double vlf = robotPower * Math.sin(robotMovementAngle + Math.PI/4) + (pivotCorrection/2);
            double vrb = robotPower * Math.sin(robotMovementAngle + Math.PI/4) - (pivotCorrection/2);
            tele.addData("power to left front wheel is ", vlf);
            tele.addData("power to right front wheel is ", vrf);
            tele.addData("power to back left wheel is ", vlb);
            tele.addData("power to back right wheel is ", vrb);
            tele.addData("Lift position", lift.getCurrentPosition());
            tele.addData("Lifting up", liftingUp);
            tele.addData("Size of stack", size);
            tele.addData("Wobble Goal X", wobbleGoalX);
            tele.addData("Wobble Goal Y", wobbleGoalY);
            telemetry.addData("Shooter velocity in ticks per seconds", shooter.getVelocity());




            /*
            double [] vals = {vlb, vrf, vlf, vrb};
            double maxVal = 0;
            for (double val : vals)
            {
                if(Math.abs(val) > maxVal)
                {
                    maxVal = val;
                }
            }
            vlb /= Math.abs(maxVal);
            vrf /= Math.abs(maxVal);
            vlf /= Math.abs(maxVal);
            vrb /= Math.abs(maxVal);


             */
            left_back.setPower(vlb * robotPower);
            right_front.setPower(vrf * robotPower);
            left_front.setPower(vlf * robotPower);
            right_back.setPower(-vrb * robotPower);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            tele.update();

            //RobotMovement.goToPosition(COUNTS_PER_INCH, tele, right_front, right_back, left_front, left_back, imu, globalPositionUpdate, 10, 10, 0.35, 0, 1.0);
        }
        left_back.setPower(0);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);


    }


    //
        /*
        This function is called at the beginning of the program to activate
        the IMU Integrated Gyro.
         */
    public void initGyro(BNO055IMU imu, HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        //telemetry.update();
    }

    public void toggleTrigger()
    {
        triggered = true;
        trigger.setPosition((TRIGGERFORWARD - 100.0) / 2420.0);
        try {
            Thread.sleep(650);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        triggered = false;
        trigger.setPosition((TRIGGERBACK - 100.0) / 2420.0);
    }

    public void toggleTriggerThrice()
    {
        triggered = true;
        trigger.setPosition((TRIGGERFORWARD-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        triggered = false;
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        triggered = true;
        trigger.setPosition((TRIGGERFORWARD-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        triggered = false;
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        triggered = true;
        trigger.setPosition((TRIGGERFORWARD-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        triggered=false;
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
    }



    public void followCurve(double COUNTS_PER_INCH, Telemetry telemetry, ArrayList<CurvePoint> allPoints, double followAngle, double allowableDistanceError, Thread liftThread)
    {

        double distanceToXTarget = allPoints.get(allPoints.size() - 1).x - globalPositionUpdate.returnXCoordinate()/ COUNTS_PER_INCH;
        double distanceToYTarget = allPoints.get(allPoints.size() - 1).y - globalPositionUpdate.returnYCoordinate()/ COUNTS_PER_INCH;
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while( opModeIsActive() && distance > allowableDistanceError)
        {
            liftThread.start();
            distanceToXTarget = allPoints.get(allPoints.size() - 1).x - globalPositionUpdate.returnXCoordinate()/ COUNTS_PER_INCH;
            distanceToYTarget = allPoints.get(allPoints.size() - 1).y - globalPositionUpdate.returnYCoordinate()/ COUNTS_PER_INCH;
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            for (int i = 0; i < allPoints.size() - 1; i++) {
                //com.company.ComputerDebugging.sendLine(new com.company.FloatPoint(allPoints.get(i).x, allPoints.get(i).y), new com.company.FloatPoint(allPoints.get(i+1).x,allPoints.get(i+1).y));
            }

            CurvePoint followMe = getFollowPointPath(allPoints, new Point(globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate()), allPoints.get(0).followDistance, globalPositionUpdate);
            //This follow distance is not always the same; write function to get follow distance; use perpendicular line
            // ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

            goToPosition(telemetry, followMe.x, followMe.y, followMe.moveSpeed, followAngle, allowableDistanceError);
        }



    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        collector = (DcMotor) hardwareMap.dcMotor.get("CollectorMandRightOdometry");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMandE");
        advancingM = (DcMotor) hardwareMap.dcMotor.get("AdvancingM");

        shooterHeights = (Servo) hardwareMap.servo.get("ShooterHeightS");
        shooterHeights2 = (Servo) hardwareMap.servo.get("ShooterHeightS2");
        block = (Servo) hardwareMap.servo.get("BlockS");


        lift = (DcMotorEx) hardwareMap.dcMotor.get("LiftM");
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        lift.setTargetPosition(0);


        PIDFCoefficients pid = new PIDFCoefficients(1000.0, 0.96, 445, 0.0, MotorControlAlgorithm.PIDF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

        PIDFCoefficients pid2 = new PIDFCoefficients(10.0, 0, 0, 0.0, MotorControlAlgorithm.PIDF);

        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid2);

        trigger = hardwareMap.servo.get("TriggerS");
        elbow = hardwareMap.servo.get("WobbleElbowS");
        hand = hardwareMap.servo.get("WobbleHandS");
        //br.setDirection(DcMotor.Direction.REVERSE);
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        //telemetry.addData("Status", "Hardware Map Init Complete");
        //telemetry.update();
    }


    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
        //Maybe substract by pi/4
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        left_front.setPower(leftPower);
        left_back.setPower(leftPower);
        right_front.setPower(rightPower);
        right_back.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }



    private class ShooterThread extends Thread
    {
        public ShooterThread()
        {
            this.setName("Shooter Thread");
        }

        @Override
        public void run() {

            if(liftingUp)
            {
                lift.setTargetPosition(HIGHPOS);
                lift.setVelocity(5000);
            }
            if(falling)
            {
                lift.setTargetPosition(0);
                lift.setVelocity(5000);
            }


        }
    }

}
