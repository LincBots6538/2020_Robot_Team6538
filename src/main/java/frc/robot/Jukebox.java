/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

/**
 * Add your docs here.
 */
public class Jukebox {

    Orchestra orchestra;
    TalonFX[] tracks = new TalonFX[4];
    String[] music_files = {"music/20thcenf.chrp",
                            "music/Cantina.chrp",
                            "music/empire.chrp",
                            "music/IMPERIAL.chrp",
                            "music/LEIATHEM.chrp",
                            "music/Starwars.chrp",
                            "music/StarwarsMed.chrp",
                            "music/SW_CERE.chrp",
                            "music/YODA.chrp"};

    public Jukebox() {
        /*for(int track = 0; track < tracks.length; track++)
        {
            tracks[track] = new TalonFX(track);
            orchestra.addInstrument(tracks[track]);
            orchestra.addInstrument(new TalonFX(track));
        }*/
    }

    public void pause() {
        orchestra.pause();
    }

    public void play(int track) {
        orchestra.loadMusic(music_files[track]);
        orchestra.play();
    }

    public void playOpener() {
        orchestra.loadMusic(music_files[0]);
        orchestra.play();
    }

    public void shufflePlay() {
        int randint = (int) (Math.random() * 7) + 1;
        String random_track = music_files[randint];
        orchestra.loadMusic(random_track);
        orchestra.play();
    }

    public void stop() {
        orchestra.stop();
    }
}
