package org.firstinspires.ftc.teamcode.teamopmodes;

import android.media.MediaPlayer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.R;

//@Disabled
@TeleOp(name="Driver Hub Sound", group="Tests")
public class DriverHubSound extends LinearOpMode {

    MediaPlayer mediaPlayer;
    private String soundPlaying = null;

    //method for clearing the media player and loading in a new sound
    private void playSound(int soundName) {
        if (mediaPlayer != null) {
            if (mediaPlayer.isPlaying()) {
                mediaPlayer.stop();
            }
            mediaPlayer.release();
            mediaPlayer = null;
            mediaPlayer = MediaPlayer.create(hardwareMap.appContext, soundName);
            mediaPlayer.start();
            gamepad1.rumbleBlips(3);
        }
    }

    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance(); //Initialises FTC dashboard on http://192.168.43.1:8080/dash

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {playSound(R.raw.rizz_sound_effect); soundPlaying = "rizz";}
            if (gamepad1.b) {playSound(R.raw.sponge_stank_noise); soundPlaying = "spongebob";}

            telemetry.addData("Sound Playing", soundPlaying);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Sound Playing", soundPlaying);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
