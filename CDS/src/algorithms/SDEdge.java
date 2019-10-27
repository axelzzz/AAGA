package algorithms;

import java.awt.Point;

/*represente une arete avec un etat et une direction*/
public class SDEdge extends Edge {

	private State state, direction;
	
	public SDEdge(Point p1, Point p2, State state, State direction) {
		
		super(p1, p2);
		this.state = state;
		this.direction = direction;
	}

	
	public State getState() { return state; } 
	public State getDirection() { return direction; } 
	
	public void setState(State s) { state = s; }
	public void setDirection(State d) { direction = d; }
}
