package algorithms;

import java.awt.Point;

public class WEdge extends Edge {
	
	private double poids;
	
	public WEdge(Point a, Point b, double dist){
		super(a, b);
		poids = dist;
	}
	
	public WEdge(Edge e){
		
		super(e.getP1(), e.getP2());
		poids=e.getDistance();
	}
	
	public double getPoids(){return poids;}
	
	public void setPoids(double poids){this.poids = poids;}
	
	public WEdge clone(){
		
		Point pbis1 = (Point) getP1().clone();
		Point pbis2 = (Point) getP2().clone();
		return new WEdge(pbis1, pbis2, pbis1.distance(pbis2));
	}
			

}

