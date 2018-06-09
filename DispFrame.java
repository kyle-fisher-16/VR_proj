package vr_demo1;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;

public class DispFrame extends JFrame{
	Float[] frame0_pts_rot = new Float[0];
	Float[] current_pts_rot = new Float[0];
	Float current_drift = 0.0f;
	Float current_angle = 0.0f;
	Float x_axis = 0.0f;
	Float y_axis = 0.0f;
	Float z_axis = 0.0f;
	
	BufferedImage bf;
	Graphics2D bfg;
	
    DispFrame(){
		setBackground(Color.BLACK);
		// set properties
		setSize(Toolkit.getDefaultToolkit().getScreenSize());
		
		setUndecorated(true);
		
		
		bf = new BufferedImage(this.getWidth(), this.getHeight(), BufferedImage.TYPE_INT_RGB);
		bfg = bf.createGraphics();
		
		setVisible(true);
	    
    }

    @Override
    public void paint(Graphics g) 
    {
    	
    		Dimension s = getSize();
    		int width = s.width;
		int height = s.height;
		int viewport_center_x = s.width / 2;
		int viewport_center_y = s.height / 2;
		
		bfg.setColor(Color.BLACK);
		bfg.fillRect(0, 0, width, height);
		
		// frame 0 kp's
		bfg.setColor(Color.WHITE);
		for (int i = 0; i < (frame0_pts_rot.length - 1); i+=2) {
			int x_scale = (int)(height * frame0_pts_rot[i] / 2.0f);
			int y_scale = (int)(height * frame0_pts_rot[i+1] / 2.0f);
			int x_pt = viewport_center_x - x_scale;
			int y_pt = viewport_center_y - y_scale;
			int radius = 3;
	    		bfg.fillOval(x_pt - radius, y_pt - radius, radius*2, radius*2);
		}
		
		// current frame kp's
		bfg.setColor(Color.RED);
		for (int i = 0; i < (current_pts_rot.length - 1); i+=2) {
			int x_scale = (int)(height * current_pts_rot[i] / 2.0f);
			int y_scale = (int)(height * current_pts_rot[i+1] / 2.0f);
			int x_pt = viewport_center_x - x_scale;
			int y_pt = viewport_center_y - y_scale;
			int radius = 3;
	    		bfg.fillOval(x_pt - radius, y_pt - radius, radius*2, radius*2);
		}
		
		// yaw text at the top
		bfg.setPaint(Color.GREEN);
        bfg.setFont(new Font("Consolas", Font.BOLD, 30));
        String str = "Total drift (yaw) corrected: " + String.format(java.util.Locale.US,"%.2f", current_drift) + "\u00b0";
        FontMetrics fm = bfg.getFontMetrics();
        int x_pt = width / 2 - fm.stringWidth(str) / 2;
        int y_pt = 35;
        bfg.drawString(str, x_pt, y_pt);
		// quaternion text
        String angle_str = String.format(java.util.Locale.US,"%.2f", current_angle);
        String x_str = String.format(java.util.Locale.US,"%.2f", x_axis);
        String y_str = String.format(java.util.Locale.US,"%.2f", y_axis);
        String z_str = String.format(java.util.Locale.US,"%.2f", z_axis);
        str = "Corrected Angle / Axis:    " + angle_str + "\u00b0 (" + x_str + " " + y_str + " "+ z_str + ")";
        x_pt = width / 2 - fm.stringWidth(str) / 2;
        y_pt = 85;
        bfg.drawString(str, x_pt, y_pt);
		
		
		
		g.drawImage(bf,0,0,null);

    }

}