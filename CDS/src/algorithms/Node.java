package algorithms;

import java.awt.Point;

public class Node {

	private Point p;
	private State s;
	private int id;
	
	public Node(Point p, State s, int id) {
		
		this.p = p;
		this.s = s;
		this.id = id;
	}
	
	
	public Point getPoint() { return p; }
	public State getState() { return s; }
	public int getId() { return id; }
	
	public void setPoint(Point p) { this.p = p; }
	public void setState(State s) { this.s = s; }
	public void setId(int i) { id = i; }
}
