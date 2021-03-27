package org.firstinspires.ftc.teamcode.Odometry;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp
public class TeleOpMode extends OpMode {
    public DcMotor bl;
    public DcMotor fl;
    public DcMotor br;
    public DcMotor fr;
    public DcMotor collector;
    public DcMotorEx shooter;
    public DcMotorEx lift;
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
    public final int TRIGGERFORWARD =  1460;
    public final int TRIGGERBACK = 1140;
    public final int HIGHSHOT = 1120;
    public final int LOWSHOT = 1650;
    public final int HIGHPOS = -1155;
    public int shootHeight = HIGHSHOT;
    public final int ELBOWUP = 900;
    public final int ELBOWDOWN = 2050;
    boolean firstTime = true;
    public int elbowH = 1300;
    public int CLOSEDPOS = 550;
    public int OPENPOS = 1575;
    public int handH = CLOSEDPOS;
    public Servo trigger;
    public Servo shooterHeights;
    public Servo shooterHeights2;
    public Servo elbow;
    public Servo hand;
    public double robotPower1 = 2.0;
    public double robotPower2 = 0.5;
    public StickyButton boostPower;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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


    @Override
    public void init() {
        fl = (DcMotor) hardwareMap.dcMotor.get("FLWheelM");
        fr = (DcMotor) hardwareMap.dcMotor.get("FRWheelandLeftOdometry");
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl = (DcMotor) hardwareMap.dcMotor.get("BLWheelM");
        br = (DcMotor) hardwareMap.dcMotor.get("BRWheelandCenterOdometry");
        //br.setDirection(DcMotorSimple.Direction.REVERSE);
        collector = (DcMotor) hardwareMap.dcMotor.get("CollectorMandRightOdometry");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMandE");
        advancingM = (DcMotorEx) hardwareMap.dcMotor.get("AdvancingM");
        shooterHeights = (Servo) hardwareMap.servo.get("ShooterHeightS");
        shooterHeights2 = (Servo) hardwareMap.servo.get("ShooterHeightS2");
        shooterHeights.setPosition((shootHeight - 100.0) / 2420.0);
        shooterHeights2.setPosition((shootHeight - 100.0) / 2420.0);
        lift = (DcMotorEx) hardwareMap.dcMotor.get("LiftM");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setVelocity(0);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        //Initialiign the sticky button to boost power
        boostPower = new StickyButton();

        //extraShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //extraShooter.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pid = new PIDFCoefficients(1000.0, 0.96, 445, 0.0, MotorControlAlgorithm.PIDF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        //extraShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        trigger = hardwareMap.servo.get("TriggerS");
        elbow = hardwareMap.servo.get("WobbleElbowS");
        hand = hardwareMap.servo.get("WobbleHandS");
        br.setDirection(DcMotor.Direction.REVERSE);
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and displays boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();
            }
        }
    }

    @Override
    public void loop() {

        if(firstTime == true)
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
                    }
                    telemetry.update();
                }
            }

            firstTime = false;
        }


        //telemetry.addData("Encoder value for lift", lift.getCurrentPosition());
        //telemetry.addData("Target encdoer value for lift", lift.getTargetPosition());


        Thread mt = (Thread) new MovementThread();
        mt.start();

        Thread st = (Thread) new ShooterThread();
        st.start();

        //Thread ct = (Thread) new CollectorThread();
        //ct.start();

        //check if left bumper is pressed
        if(gamepad1.left_bumper || gamepad2.left_bumper)
        {
            toggleTrigger();
        }

        if(gamepad1.right_bumper || gamepad2.right_bumper)
        {
            toggleTriggerThrice();
        }



        if(gamepad1.left_trigger != 1.0 && gamepad1.right_trigger != 1.0)
        {
            collector.setPower(0.00);
        }
        if(gamepad1.left_trigger >= 0.1)
        {
            //collectorVal = 1;
            collector.setPower(1.00);
        }
        if(gamepad1.right_trigger >= 0.1) {
            //collectorVal = -1;
            collector.setPower(-1.00);
        }

        if(gamepad2.right_trigger >= 0.1)
        {
            shootHeight += 5;
        }

        if(gamepad2.left_trigger >= 0.1)
        {
            shootHeight -= 5;
        }

        if(gamepad1.right_stick_button)
        {
            robotPower1 = 2.0;
        }
        if(gamepad1.left_stick_button)
        {
            robotPower1 = 0.5;
        }
        if(gamepad2.right_stick_button)
        {
            robotPower2 = 1.0;
        }
        if(gamepad2.left_stick_button)
        {
            robotPower2 = 0.5;
        }


        if(gamepad1.x || gamepad2.x)
        {
            shootHeight = HIGHSHOT;
        }
        if(gamepad1.b || gamepad2.b)
        {
            shootHeight = LOWSHOT;
        }

        if(gamepad1.y || gamepad2.y)
        {
            liftingUp = true;
            falling = false;
                //topHeight = lift.getCurrentPosition() - 100;

        }

        if(gamepad1.a || gamepad2.a)
        {
            falling = true;
            liftingUp = false;
            //bottomHeight = lift.getCurrentPosition() + 100;
        }

        if(gamepad1.dpad_up || gamepad2.dpad_up)
        {
            elbowH = ELBOWUP;
        }

        if(gamepad1.dpad_down || gamepad2.dpad_down)
        {
            elbowH = ELBOWDOWN;
        }

        if(gamepad1.dpad_left || gamepad2.dpad_left)
        {
            if(handH == OPENPOS)
            {
                handH = CLOSEDPOS;
            }
            else if(handH == CLOSEDPOS)
            {
                handH = OPENPOS;
            }
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(gamepad1.dpad_right || gamepad2.dpad_right)
        {
            if(handH == OPENPOS)
            {
                handH = CLOSEDPOS;
            }
            else if(handH == CLOSEDPOS)
            {
                handH = OPENPOS;
            }
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        telemetry.addData("Lifting up", liftingUp);
        //telemetry.addData("Top Height", topHeight);
        //telemetry.addData("Bottom Height", bottomHeight);
        telemetry.addData("Falling down", falling);
        telemetry.addData("Shooter toggle", shooterToggle);
        telemetry.addData("Hand H", handH);
        telemetry.addData("Trigger position", trigger.getPosition());
        telemetry.addData("Trigger bool", triggered);
        telemetry.addData("Shooter height", shooterHeights.getPosition());
        telemetry.addData("Shooter velocity in ticks per seconds", shooter.getVelocity());
        telemetry.addData("PID Coef", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("PID Coef - P", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        telemetry.addData("Robot Power", robotPower1);

        telemetry.update();
    }

    /*

    private void fall() {
        lift.setPower(0.5);
        lift.setTargetPosition(1300);
        lift.setPower(0.05);
    }

    private void lift() {
        lift.setPower(0.5);
        lift.setTargetPosition(0);
        lift.setPower(0.05);
    }

    */

    public void toggleShooter()
    {
        if(shooterToggle == false)
        {
            shooterToggle = true;
        }
        else if(shooterToggle == true)
        {
            shooterToggle = false;
        }
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

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

        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        trigger.setPosition((TRIGGERFORWARD-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        trigger.setPosition((TRIGGERFORWARD-100.0)/ 2420.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        trigger.setPosition((TRIGGERBACK-100.0)/ 2420.0);
        triggered=false;



    //triggered = false;
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

    private class ShooterThread extends Thread
    {
        public ShooterThread()
        {
            this.setName("Shooter Thread");
        }

        @Override
        public void run() {
            if(shooterToggle == true)
            {
                shooter.setVelocity(1500);
                advancingM.setPower(-0.80);
            }
            else if(shooterToggle == false)
            {
                shooter.setVelocity(1500);
                advancingM.setPower(-0.80);
            }

            if(liftingUp)
            {
                lift.setTargetPosition(HIGHPOS);
                lift.setVelocity(2000);
            }
            if(falling)
            {
                lift.setTargetPosition(0);
                lift.setVelocity(2000);
            }

            elbow.setPosition((elbowH -100.0) / 2420.0);
            hand.setPosition((handH -100.0) / 2420.0);
            shooterHeights.setPosition((shootHeight-100.0)/ 2420.0);
            shooterHeights2.setPosition((shootHeight-100.0)/ 2420.0);


        }
    }


/*
    private class CollectorThread extends Thread
    {
        public CollectorThread()
        {
            this.setName("Collector Thread");
        }

        @Override
        public void run() {
            if(collectorVal == 1.0)
            {
                collector.setPower(1.00);
            }
            else if(collectorVal == -1.0)
            {
                collector.setPower(-1.00);
            }
        }
    }
    */

    private class MovementThread extends Thread
    {
        public MovementThread()
        {
            this.setName("DriveThread");

        }

        public void run()
        {
            if(gamepad1.left_stick_x != 0.0 || gamepad1.right_stick_x != 0.0 || gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_y != 0.0)
            {
                try {
                    double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                    double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                    double rightX = gamepad1.right_stick_x;
                    final double v1 = r * Math.cos(robotAngle) + rightX;
                    final double v2 = r * Math.sin(robotAngle) - rightX;
                    final double v3 = r * Math.sin(robotAngle) + rightX;
                    final double v4 = r * Math.cos(robotAngle) - rightX;

                    fl.setPower(robotPower1*v1);
                    fr.setPower(robotPower1*v2);
                    bl.setPower(robotPower1*v3);
                    br.setPower(-robotPower1*v4);
                }
                catch (Exception e)
                {
                    telemetry.addData("Exception", e);
                    telemetry.update();
                }
            }

            else if (gamepad2.left_stick_x != 0.0 || gamepad2.right_stick_x != 0.0 || gamepad2.left_stick_y != 0.0 || gamepad2.right_stick_y != 0.0)
            {
                try {
                    double r = Math.hypot(gamepad2.left_stick_x, -gamepad2.left_stick_y);
                    double robotAngle = Math.atan2(-gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4;
                    double rightX = gamepad2.right_stick_x;
                    final double v1 = r * Math.cos(robotAngle) + rightX;
                    final double v2 = r * Math.sin(robotAngle) - rightX;
                    final double v3 = r * Math.sin(robotAngle) + rightX;
                    final double v4 = r * Math.cos(robotAngle) - rightX;

                    fl.setPower(robotPower2*v1);
                    fr.setPower(robotPower2*v2);
                    bl.setPower(robotPower2*v3);
                    br.setPower(-robotPower2*v4);
                }
                catch (Exception e)
                {
                    telemetry.addData("Exception", e);
                    telemetry.update();
                }
            }
            else
            {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }

        }

    }
}
