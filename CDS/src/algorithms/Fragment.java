package algorithms;

import java.util.ArrayList;

public class Fragment {
	
	private Node candidate;
	private ArrayList<Node> domain;
	
	public Fragment(Node candidate) {
		
		this.candidate = candidate;
		domain = new ArrayList<Node>();
	}
	
	
	public Node getCandidate() { return candidate; }	
	public ArrayList<Node> getDomain() { return domain; }
	
	public void setCandidate(Node p) { candidate = p; }
	public void setDomain(ArrayList<Node> d) { domain = d; }
	
	public int identity() { return domain.size() + 1; }
	
}
