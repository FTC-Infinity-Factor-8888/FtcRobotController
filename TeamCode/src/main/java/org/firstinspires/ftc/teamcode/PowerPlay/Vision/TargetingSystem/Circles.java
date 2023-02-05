package org.firstinspires.ftc.teamcode.PowerPlay.Vision.TargetingSystem;

import org.opencv.core.Core;

public class Circles {
	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		new TargetingComputer().run(args);
	}
}
