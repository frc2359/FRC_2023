package frc.robot.subsystems;

import javax.swing.JFrame;
import javax.swing.JPanel;

import java.awt.Graphics;


class AprilTagGUI extends JPanel {
    
    JFrame frame = new JFrame();

    int[] ll = {250, 250};

    public static void main(String[] args){
        AprilTagGUI gui = new AprilTagGUI();
    }

    public AprilTagGUI() {
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setBounds(30, 30, 530, 530);

        frame.add(this);

        frame.setVisible(true);
    }

    @Override
    public void paint(Graphics g){
        super.paint(g);

        g.drawRect(240, 20, 20, 20);

        g.drawOval(ll[0]-20, ll[1]-20, 40, 40);
    }

    public void setX(int x){
        ll[0] = x;
        this.repaint();
    }

    public void setY(int y){
        ll[1] = y;
        this.repaint();
    }

    public void setPos(double distance, double angle){
        
    }

}
