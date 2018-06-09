package vr_demo1;

import java.awt.Color;
import java.awt.Toolkit;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;

import javax.swing.JFrame;

import javax.swing.*;
import java.awt.*;

public class demo_loop {
	
	static String addr = "http://169.254.91.166:8000";

	public static Float[] getAngleStats() {
		int num_stats = 5;
		Float[] res = new Float[num_stats];
		try {
			URL url = new URL(addr + "?type=angle-stats");
	        URLConnection yc = url.openConnection();
	        BufferedReader in = new BufferedReader(
	                                new InputStreamReader(
	                                yc.getInputStream()));
	        String inputLine;
	        while ((inputLine = in.readLine()) != null) {
	        		if (inputLine.length() > 0) {
	        			String[] ss = inputLine.trim().split(" ");
	        			for (int i = 0; i < num_stats; i++) {
	        				res[i] = Float.parseFloat(ss[i]);
	        			}
	        		}
	        }
	        in.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return res;
	}
	
	public static Float[] getRotMat() {
		Float[] mat = new Float[9];
		try {
			URL url = new URL(addr + "?type=imu-fused-mat");
	        URLConnection yc = url.openConnection();
	        BufferedReader in = new BufferedReader(
	                                new InputStreamReader(
	                                yc.getInputStream()));
	        String inputLine;
	        while ((inputLine = in.readLine()) != null) {
	        		if (inputLine.length() > 0) {
	        			String[] ss = inputLine.trim().split(" ");
	        			for (int i = 0; i < 9; i++) {
	        				mat[i] = Float.parseFloat(ss[i]);
	        			}
	        		}
	        }
	        in.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return mat;
	}
	
	public static Float[] getKeypoints(String name) {
		// Get the Frame0 points
		Float[] frame0pts = new Float[0];
		try {
			URL url = new URL(addr + "?type="+name);
	        URLConnection yc = url.openConnection();
	        BufferedReader in = new BufferedReader(
	                                new InputStreamReader(
	                                yc.getInputStream()));
	        String inputLine;
	        while ((inputLine = in.readLine()) != null) {
	        		if (inputLine.length() > 0) {
	        			String[] ss = inputLine.trim().split(" ");
	        			frame0pts = new Float[ss.length];
	        			for (int i = 0; i < ss.length; i++) {
	        				frame0pts[i] = Float.parseFloat(ss[i]);
	        			}
	        		}
	        }
	        in.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return frame0pts;
	}

	public static void main(String[] args) {
		// TODO Auto-generated method stub
        DispFrame df = new DispFrame();
        
//		frame.setVisible(true);
		
		
		Float[] frame0_pts = getKeypoints("frame0-kp");
		Float[] current_pts = getKeypoints("current-kp");
		Float[] angleStats = getAngleStats();
		
//		df.repaint(); 
		
		int ct = 0;
		while(true) {
			if (ct % 50 == 0) {
				current_pts = getKeypoints("current-kp");
				ct = 0;
			}
			if (ct % 5 == 0) {
				angleStats = getAngleStats();
			}
			Float[] rot = getRotMat();
			df.frame0_pts_rot = applyRot(rot, frame0_pts);
			df.current_pts_rot = applyRot(rot, current_pts);
			df.current_drift = angleStats[0];
			df.current_angle = angleStats[1];
			df.x_axis = angleStats[2];
			df.y_axis = angleStats[3];
			df.z_axis = angleStats[4];
			df.repaint(); 
			ct += 1;
		}
		
		
	}
	
	public static Float[] applyRot(Float[] rot, Float[] xy) {
		Float[] out = new Float[xy.length]; 
		for (int i = 0; i < (xy.length - 1); i+=2) {
			float x = xy[i];
			float y = xy[i+1];
			float x_p = rot[0] * x + rot[1] * y + rot[2];
			float y_p = rot[3] * x + rot[4] * y + rot[5];
			float z_p = rot[6] * x + rot[7] * y + rot[8];
			if (z_p > 0.0) {
				out[i] = x_p / z_p;
				out[i+1] = y_p / z_p;
			}
			else {
				out[i] = 100.0f; // draw these off screen
				out[i+1] = 100.0f; // draw these off screen
			}
		}
		return out;
	}

}
