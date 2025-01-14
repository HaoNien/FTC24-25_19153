package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import androidx.core.content.pm.PermissionInfoCompat;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

@Config
public abstract class robotBase extends OpMode {
    protected SampleMecanumDrive drive;

    protected DcMotorEx slideL;
    protected DcMotorEx slideR;


    //protected CRServo FrontL, FrontR;
    protected Servo trainL, trainR;
    protected Servo f1, f2,f3,f4;
    protected ServoImplEx bl1, br1;

    protected Servo b2, b3,b4;

    private PIDController SlidePID = new PIDController(0, 0, 0);

    /*------------------Slide_PIDF-----------------------*/
    public static double slideP = 0.5;
    public static double slideI = 0;
    public static double slideD = 0;

    public static double slideP_hang = 1.5;
    public static double slide_f_coeff = 0;

    public static double slide_motorEnc = 103.8;
    public static double slide_Ratio = 1 / 1.4;

    public static double slide2lenth = slide_motorEnc * slide_Ratio / 0.8;
    public static double smax = 98;
    public static double smax0 = 76;

    public static double smin = 40;
    public double slidePower;

    public static double slidePosNow = 0;

    public double slideTarget = 0;

    public static double f1_pos = 0.5;

    public static double f1_OFFSET = 2.0;

    public static double f2_pos = 0.5;//flat0.89

    public static double f2_OFFSET = 0.0;
    public static double f3_pos = 0.3
            ;
    public static double f4_pos = 0.5;//0.31close 0.65open

    public static double b1_pos = 0.8;//08 low
    //public static double br1_pos = bl1_pos;

    public static double b2_pos = 0.5;
    public static double b3_pos = 0.5;
    public static double b4_pos = 0.5;


    public static double trainl_pos = 0.5;//0.22 IN 0.9OUT
    public static double trainr_pos = 0.5;//0.9in   0.12OUT



    public boolean isHangingMode = false; // 預設為普通模式
    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 1000)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 1000)  //  Pause for 300 mSec

            .build();


    public boolean initDone=false;

    @Override
    public void init(){

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");



        slideL.setDirection(DcMotorSimple.Direction.REVERSE);

        trainL = hardwareMap.get(ServoImplEx.class, "trainL");
        trainR = hardwareMap.get(ServoImplEx.class, "trainR");
        //PwmControl.PwmRange axon=new PwmControl.PwmRange(500,2500);
        f1 = hardwareMap.get(Servo.class, "f1");
        f2 = hardwareMap.get(Servo.class, "f2");
        f3 = hardwareMap.get(Servo.class, "f3");
        f4 = hardwareMap.get(Servo.class, "f4");

        bl1 = hardwareMap.get(ServoImplEx.class, "bl1");
        br1 = hardwareMap.get(ServoImplEx.class, "br1");
        br1.setDirection(Servo.Direction.REVERSE);
        //bl1.setPwmRange(new PwmControl.PwmRange(500,2500));

        b2 = hardwareMap.get(Servo.class, "b2");
        b3 = hardwareMap.get(Servo.class, "b3");
        b4 = hardwareMap.get(Servo.class, "b4");


        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setPower(0);

        slideR.setPower(0);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robotInit();
        initDone = true;
        telemetry.addData("init","done");
        telemetry.update();
    }
    public void init_loop(){
        if(initDone){
            robotInitLoop();
        }
    }
    public void loop(){
        robotStart();

    }


    protected abstract void robotInit();
    protected abstract void robotInitLoop();
    protected abstract void robotStart();
    public void trainToPosition(double pos) {
        // 確保傳入位置在 0.2 - 0.9 之間
        pos = Math.max(0.2, Math.min(pos, 0.9));

        // 將 0.2 - 0.9 映射到 0.0 - 1.0
        double mappedPos = (pos - 0.2) / (0.9 - 0.2);

        // 左右相反方向控制
        trainl_pos = mappedPos;          // 左側滑軌位置
        trainr_pos = 1.0 - mappedPos;    // 右側滑軌位置 (反向)

        // 設置伺服馬達位置
        trainL.setPosition(trainl_pos);
        trainR.setPosition(trainr_pos);
    }


    public void slideToPosition(double slidePos) {

        slidePosNow = (slideL.getCurrentPosition() / slide2lenth) * 2 + smin;


        double slidekp;
        if (isHangingMode) {
            slidekp = slideP_hang;
        } else
            slidekp = slideP;

        SlidePID.setPID(slidekp, slideI, slideD);
        slidePower = SlidePID.calculate(slidePosNow, slidePos);
        slideL.setPower(slidePower);
        slideR.setPower(slidePower);

    }


    public void fclawopen (){
        f4.setPosition(0.65);
    }
    public void fclawclose (){
        f4.setPosition(0.31);
    }

    // 設定角度方法，0.5 為 0 度，伺服角度總行程 276.923 度
    public void f1turn(double angle) {
        // 定義 OFFSET，可根據需要進行調整 (例如：-5.0 修正)

        // 修正後的目標角度
        double correctedAngle = angle + f1_OFFSET;

        // 確保角度在合法範圍內
        correctedAngle = Math.max(-276.923 / 2, Math.min(correctedAngle, 276.923 / 2));

        // 將角度映射到伺服輸入範圍 (0.0 - 1.0)
        double servoPos = 0.5 - (correctedAngle / 276.923);

        // 設置伺服位置
        f1.setPosition(servoPos);
    }
    public void f2turn(double angle) {
        // 定義 OFFSET，可根據需要進行調整 (例如：-5.0 修正)

        // 修正後的目標角度
        double correctedAngle = angle + f2_OFFSET;

        // 確保角度在合法範圍內
        correctedAngle = Math.max(-20, Math.min(correctedAngle, 160));

        // 將角度映射到伺服輸入範圍 (0.0 - 1.0)
        double servoPos = 0.89 - (correctedAngle / 180);

        // 設置伺服位置
        f2.setPosition(servoPos);


    }


}
