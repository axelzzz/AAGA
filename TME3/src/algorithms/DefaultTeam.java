package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

public class DefaultTeam {
	
	
   public ArrayList<Point> neighbor(Point p, ArrayList<Point> vertices, int edgeThreshold){
		    ArrayList<Point> result = new ArrayList<Point>();

		    for (Point point:vertices) if (point.distance(p)<edgeThreshold && !point.equals(p)) result.add((Point)point.clone());

		    return result;
	 }
   
	
	public int nbVoisinsOfPoint(Point p, ArrayList<Point> points, int edgeThreshold) {
		  return neighbor(p, points, edgeThreshold).size();
	}
	
	
	 
	 public Point pointDegreMaxList(ArrayList<Point> points, int edgeThreshold) {
		 
		 Point max = points.get(0);
		 
		 for(int i = 1 ; i < points.size() ; i++) {
			 Point current = points.get(i);
			 if(neighbor(max, points, edgeThreshold).size() < neighbor(current, points, edgeThreshold).size() )
				 max = current;
		 }
		 
		 return max;
	 }
	 
	
	 

	 public Point pointDegreMinList(ArrayList<Point> points, int edgeThreshold) {
		 
		 Point min = points.get(0);
		 
		 for(int i = 1 ; i < points.size() ; i++) {
			 Point current = points.get(i);
			 if(neighbor(min, points, edgeThreshold).size() > neighbor(current, points, edgeThreshold).size() )
				 min = current;
		 }
		 
		 return min;
	 }
	
	
	 
	 public ArrayList<Point> supprimerDoublons(ArrayList<Point> origins) {
		  
	      Set set = new HashSet() ;
	      set.addAll(origins) ;
	      ArrayList<Point> res = new ArrayList<>(set) ;	
	      //System.out.println("size list after supp doublons "+res.size());
	      return res;
			  
	  }
	 
	 
	 

	 
	 public ArrayList<Point> reste(ArrayList<Point> points, ArrayList<Point> toRemove) {
		  
		  ArrayList<Point> res = new ArrayList<>();
		  res.addAll(points);
		  
		  for(Point p:toRemove)
			  res.remove(p);
		  
		  return res;
	  }
 
	  
	  public Point clonePoint(Point toClone) {
		  
		  return new Point(toClone.x, toClone.y);
	  }
	  
	  
	  public ArrayList<Point> cloneList(ArrayList<Point> toClone) {
		  
		  ArrayList<Point> res = new ArrayList<>();
		  
		  for(Point p:toClone)
			  res.add(clonePoint(p));
		  
		  return res;
	  }
	  
	  
	  public boolean equals(Point a, Point b) {
		  
		  if (a!= null && b != null) {
			  if(a.x == b.x && a.y == b.y)
				  return true;
			  return false;
			  
		  }
		  return false;
	  }
	  
	

	  public boolean isDominant(ArrayList<Point> origins,
			  					ArrayList<Point> toTest,
			  					int edgeThreshold) {
		  
		  ArrayList<Point> clone = cloneList(origins);
		  
		  for(int i = 0 ; i < toTest.size() ; i++) {
			  Point p = toTest.get(i);
			  clone.remove(p);
			  clone.removeAll(neighbor(p, origins, edgeThreshold));
		  }
		  
		  return clone.size() == 0;
	  }
	  
	 
	  public ArrayList<Point> neighborsOfPoints(ArrayList<Point> points, ArrayList<Point> origins, int edgeThreshold) {
		  
		  ArrayList<Point> res = new ArrayList<>();
		  
		  for(Point p:points)
			  res.addAll(neighbor(p, origins, edgeThreshold));
		  return res;
	  }
	  
	  
	  
	  public boolean estVoisinDePersonne(Point a, ArrayList<Point> points, int edgeThreshold) {
			for(Point p:points) {
				if(a.distance(p) <= edgeThreshold) {/*System.out.println("a voisin");*/return false;}
			}
			//System.out.println("n'a pas de voisin");
			return true;
		}
	  
	  
	  //on parcourt dans l'ordre, sans re trier
	  public ArrayList<Point> getStable(ArrayList<Point> points, int edgeThreshold){
			
			ArrayList<Point> res = new ArrayList<>();		
			
			res.add(points.get(0));
			for(Point p:points) {			
				if(estVoisinDePersonne(p, res, edgeThreshold))res.add(p);		
			}		
			return res;		
		}
	  
	
	  public ArrayList<Point> getDSRandomInverse(ArrayList<Point> points, int edgeThreshold) {
		  
		  ArrayList<Point> tmp = cloneList(points);
		  ArrayList<Point> res = new ArrayList<>();	
		  
		  Random r = new Random();
		  
		  while(!isDominant(points, res, edgeThreshold)) {
			  Point randomP = tmp.remove(r.nextInt(tmp.size()));
			  ArrayList<Point> voisinsP = neighbor(randomP, tmp, edgeThreshold);
			  res.add(randomP);
			  
			  tmp.removeAll(voisinsP);
			 
		  }
		 
		  
		  return res;
	  }
	  
	  
	  
	  
	  
  public ArrayList<Point> getDSRandom(ArrayList<Point> points, int edgeThreshold) {
		  
		  ArrayList<Point> tmp = cloneList(points);
		  ArrayList<Point> res = cloneList(points);	
		  
		  Random r = new Random();
		  
		  while(isDominant(points, res, edgeThreshold) && tmp.size() > 0 ) {
			  Point randomP = tmp.remove(r.nextInt(tmp.size()));
			  ArrayList<Point> voisinsP = neighbor(randomP, tmp, edgeThreshold);
			  res.removeAll(voisinsP);
			  
			  tmp.removeAll(voisinsP);
			 
		  }
		 
		  
		  return res;
	  }
  
  
	  
	  
	  public ArrayList<Point> getDSGloutonDMaxInverse(ArrayList<Point> origins, int edgeThreshold) {
		  
		  ArrayList<Point> tmp = cloneList(origins);
		  ArrayList<Point> res = new ArrayList<>();	
		  
		  //on recup le point de degmax
		  Point maxdeg = pointDegreMaxList(tmp, edgeThreshold);
		  //on l ajoute dans l ens dom
		  res.add(maxdeg);
		  ArrayList<Point> voisinsP = neighbor(maxdeg, tmp, edgeThreshold);
		  //on garde le reste
		  tmp.removeAll(voisinsP);
		  tmp.remove(maxdeg);
		 
		  
		  while(!isDominant(origins, res, edgeThreshold) ) {
			  
			
			  maxdeg = pointDegreMaxList(tmp, edgeThreshold);
			  voisinsP = neighbor(maxdeg, tmp, edgeThreshold);
			  
			  res.add(maxdeg);
			  tmp.removeAll(voisinsP);
			  tmp.remove(maxdeg);
	
		  }
		  
		  return res;
	  }
	  
	  
	  
	  public ArrayList<Point> getDSGloutonDMax(ArrayList<Point> origins, int edgeThreshold) {
		  
		  ArrayList<Point> tmp = cloneList(origins);
		  ArrayList<Point> res = cloneList(origins);	
		  
		  //on recup le point de degmax
		  Point maxdeg = pointDegreMaxList(tmp, edgeThreshold);
		
		 
		  ArrayList<Point> voisinsP = neighbor(maxdeg, tmp, edgeThreshold);
		 
		  res.removeAll(voisinsP);
		  
		  tmp.removeAll(voisinsP);
		  tmp.remove(maxdeg);
		  
		  
		  while(isDominant(origins, res, edgeThreshold) && tmp.size() > 0 ) {
			  

			  maxdeg = pointDegreMaxList(tmp, edgeThreshold);
			  voisinsP = neighbor(maxdeg, tmp, edgeThreshold);
			  
			  res.removeAll(voisinsP);
			  tmp.removeAll(voisinsP);
			  tmp.remove(maxdeg);
			 
		  }
		
		  
		  return res;
	  }  
	  

	//remplacer 2 points vivants par 1 point mort
	  public ArrayList<Point> localSearchV1(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
		  
		  Random rand = new Random();
	
		  Point p = solution.remove(rand.nextInt(solution.size()));
		  Point q = solution.remove(rand.nextInt(solution.size()));
		  
		  Point r = reste.remove(rand.nextInt(reste.size()));
		  
		  solution.add(r);
		  
		  if(isDominant(origins, solution, edgeThreshold)) {
			  System.out.println("is valid");
			  return solution;
		  }
		  else {
			  
			  reste.add(r);
			  solution.add(p);
			  solution.add(q);
			  solution.remove(r);
			  return solution;
		  }
			  
	  }
	  
	//remplacer 3 points vivants par 2 points morts
	  public ArrayList<Point> localSearchV2(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
		  
		 
		  Random rand = new Random();
			
		  Point p = solution.remove(rand.nextInt(solution.size()));
		  Point q = solution.remove(rand.nextInt(solution.size()));
		  Point a = solution.remove(rand.nextInt(solution.size()));
		  
		  Point r = reste.remove(rand.nextInt(reste.size()));
		  Point s = reste.remove(rand.nextInt(reste.size()));
		  
		  solution.add(r);
		  solution.add(s);
		  
		  if(isDominant(origins, solution, edgeThreshold)) {
			  System.out.println("is valid");
			  return solution;
		  }
		  else {
			  
			  reste.add(r);
			  reste.add(s);
			  solution.add(p);
			  solution.add(q);
			  solution.add(a);
			  solution.remove(r);
			  solution.remove(s);
			  return solution;
		  }	  
	  }
	  
	//remplacer 3 points vivants par 1 point mort
	  public ArrayList<Point> localSearchV3(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
		  
		  Random rand = new Random();
			
		  Point p = solution.remove(rand.nextInt(solution.size()));
		  Point q = solution.remove(rand.nextInt(solution.size()));
		  Point a = solution.remove(rand.nextInt(solution.size()));
		  
		  Point r = reste.remove(rand.nextInt(reste.size()));
		  
		  
		  solution.add(r);
		  
		  
		  if(isDominant(origins, solution, edgeThreshold)) {
			  System.out.println("is valid");
			  return solution;
		  }
		  else {
			  
			  reste.add(r);
			  
			  solution.add(p);
			  solution.add(q);
			  solution.add(a);
			  solution.remove(r);
			  
			  return solution;
		  }	 
			  
	  }
	  
	//remplacer 4 points vivants par 3 point mort
	  public ArrayList<Point> localSearchV4(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
		  
	

		  Random rand = new Random();
			
		  Point p = solution.remove(rand.nextInt(solution.size()));
		  Point q = solution.remove(rand.nextInt(solution.size()));
		  Point a = solution.remove(rand.nextInt(solution.size()));
		  Point b = solution.remove(rand.nextInt(solution.size()));
		  
		  Point r = reste.remove(rand.nextInt(reste.size()));
		  Point s = reste.remove(rand.nextInt(reste.size()));
		  Point t = reste.remove(rand.nextInt(reste.size()));
		  
		  solution.add(r);
		  solution.add(s);
		  solution.add(t);
		  
		  if(isDominant(origins, solution, edgeThreshold)) {
			  System.out.println("is valid");
			  return solution;
		  }
		  else {
			  
			  reste.add(r);
			  reste.add(s);
			  reste.add(t);
			  solution.add(p);
			  solution.add(q);
			  solution.add(a);
			  solution.add(b);
			  solution.remove(r);
			  solution.remove(s);
			  solution.remove(t);
			  return solution;
		  }	  
			  
	  }
	  
	//remplacer 4 points vivants par 2 point mort
	  public ArrayList<Point> localSearchV5(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
		  
		  
		  Random rand = new Random();
			
		  Point p = solution.remove(rand.nextInt(solution.size()));
		  Point q = solution.remove(rand.nextInt(solution.size()));
		  Point a = solution.remove(rand.nextInt(solution.size()));
		  Point b = solution.remove(rand.nextInt(solution.size()));
		  
		  Point r = reste.remove(rand.nextInt(reste.size()));
		  Point s = reste.remove(rand.nextInt(reste.size()));
		 
		  
		  solution.add(r);
		  solution.add(s);
		  
		  
		  if(isDominant(origins, solution, edgeThreshold)) {
			  System.out.println("is valid");
			  return solution;
		  }
		  else {
			  
			  reste.add(r);
			  reste.add(s);
			  
			  solution.add(p);
			  solution.add(q);
			  solution.add(a);
			  solution.add(b);
			  solution.remove(r);
			  solution.remove(s);
			 
			  return solution;
		  }	 
			  
	  }
	  
	  
	  
	//remplacer 1 point vivant par 1 point mort
	  public ArrayList<Point> permutation(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
		
		  Random rand = new Random();
		  
		  Point p = solution.remove(rand.nextInt(solution.size()));
		  Point q = reste.remove(rand.nextInt(reste.size()));
		  
		  solution.add(q);
		  	
		  if(isDominant(origins, solution, edgeThreshold)) 			  
			  return solution;
		  
		  else {
			  
			  solution.add(p);
			  solution.remove(q);
			  reste.add(q);
			  return solution;
		  }
			  
	  }
	  
	  
	  
	  
	  public ArrayList<Point> improve(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> rest, int edgeThreshold, int nbIte) {
		  
		  ArrayList<Point> res = localSearchV1(origins, solution, rest, edgeThreshold);
		  
		  for(int i = 0 ; i < nbIte ; i++) {
			  
			  res = localSearchV1(origins, res, reste(origins, res), edgeThreshold);
			  permutation(origins, res, reste(origins, res), edgeThreshold);
			  if(i%20000 == 0)
			  	System.out.println("taille "+res.size());
		  }
		  
		  return res;
		  
	  }
	  
	  
	  
	  public ArrayList<Point> improveGloutonRandom(ArrayList<Point> origins, int edgeThreshold, int nbIte) {
		  
		  ArrayList<Point> res = getDSRandom(origins, edgeThreshold);
		  ArrayList<Point> tmp;
		  
		  for(int i = 0 ; i < nbIte ; i++) {
			  
			  tmp = res;
			  res = getDSRandom(origins, edgeThreshold);
			  
			  if(tmp.size() < res.size())
				  res = tmp;
				  
			  
		  }
		  
		  return res;
		  
	  }
	 
	 
	 
  public ArrayList<Point> calculDominatingSet(ArrayList<Point> points, int edgeThreshold) {
	  
	  points = supprimerDoublons(points);
	  ArrayList<Point> resGlouton; 
	  
	  System.out.println("processing...");
	  //resGlouton = getDSRandom(points, edgeThreshold);
	  //resGlouton = improveGloutonRandom(points, edgeThreshold, 100);
	  resGlouton = getDSGloutonDMaxInverse(points, edgeThreshold);
	  ArrayList<Point> morts = reste(points, resGlouton);
	  resGlouton = improve(points, resGlouton, morts, edgeThreshold, 200000);
	  System.out.println("end");
	  
    return resGlouton;
  }
  
  
  
  
  
  //FILE PRINTER
  private void saveToFile(String filename,ArrayList<Point> result){
    int index=0;
    try {
      while(true){
        BufferedReader input = new BufferedReader(new InputStreamReader(new FileInputStream(filename+Integer.toString(index)+".points")));
        try {
          input.close();
        } catch (IOException e) {
          System.err.println("I/O exception: unable to close "+filename+Integer.toString(index)+".points");
        }
        index++;
      }
    } catch (FileNotFoundException e) {
      printToFile(filename+Integer.toString(index)+".points",result);
    }
  }
  private void printToFile(String filename,ArrayList<Point> points){
    try {
      PrintStream output = new PrintStream(new FileOutputStream(filename));
      int x,y;
      for (Point p:points) output.println(Integer.toString((int)p.getX())+" "+Integer.toString((int)p.getY()));
      output.close();
    } catch (FileNotFoundException e) {
      System.err.println("I/O exception: unable to create "+filename);
    }
  }

  //FILE LOADER
  private ArrayList<Point> readFromFile(String filename) {
    String line;
    String[] coordinates;
    ArrayList<Point> points=new ArrayList<Point>();
    try {
      BufferedReader input = new BufferedReader(
          new InputStreamReader(new FileInputStream(filename))
          );
      try {
        while ((line=input.readLine())!=null) {
          coordinates=line.split("\\s+");
          points.add(new Point(Integer.parseInt(coordinates[0]),
                Integer.parseInt(coordinates[1])));
        }
      } catch (IOException e) {
        System.err.println("Exception: interrupted I/O.");
      } finally {
        try {
          input.close();
        } catch (IOException e) {
          System.err.println("I/O exception: unable to close "+filename);
        }
      }
    } catch (FileNotFoundException e) {
      System.err.println("Input file not found.");
    }
    return points;
  }
  
}
