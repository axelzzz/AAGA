package algorithms;

import java.awt.Point;

public class Edge {	
	
	private Point p1;
	private Point p2;	
	
	public Edge(Point p1, Point p2){
		this.p1=p1;
		this.p2=p2;
	}
	
	public Point getP1() {return p1;}
	
	public Point getP2() {return p2;}
	
	public Edge clone(){
		
		Point pbis1 = (Point) p1.clone();
		Point pbis2 = (Point) p2.clone();
		return new Edge(pbis1, pbis2);
	}	

	public boolean egal(Edge e){
		if(this==e){return true;}
		if(e == null){return false;}
		if(this.getClass() != e.getClass()){return false;}
		if(e.getP1().getX() == this.getP1().getX() && e.getP1().getY() == this.getP1().getY()
				&& e.getP2().getX() == this.getP2().getX() && e.getP2().getY() == this.getP2().getY()){
			return true;
		}
		return false;
	}
	
	public void setEdge(Point a, Point b) {
		p1.setLocation(a.getX(), a.getY());
		p2.setLocation(b.getX(), b.getY());
	}
	
	public double getDistance() {return p1.distance(p2);}	
		

}