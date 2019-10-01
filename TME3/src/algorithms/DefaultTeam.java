package algorithms;

import java.awt.Point;
import java.util.ArrayList;
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
	
	Evaluation e = new Evaluation();
	
	
	public int nbVoisinsOfPoint(Point p, ArrayList<Point> points, int edgeThreshold) {
		  return e.neighbor(p, points, edgeThreshold).size();
	}
	
	
	public void afficherListeDegres(ArrayList<Point> points, LinkedHashMap<Point,Integer> map) {
		  
		  for(Point p:points) 
			  System.out.println("deg of "+p+" : "+map.get(p));
		  
	  }
	
	
	public LinkedHashMap<Point,Integer> initMapAssocPtsDeg(ArrayList<Point> points, int edgeThreshold) {
		  
		  LinkedHashMap<Point,Integer> assocPointsDegres = new LinkedHashMap<Point, Integer>();
		  
		  for(Point p:points) {
			  assocPointsDegres.put( p, Integer.valueOf( nbVoisinsOfPoint(p, points, edgeThreshold) ) );
		  }
		  
		  return assocPointsDegres;
	  }
	
	
	 public void afficherMapDegres(LinkedHashMap<Point,Integer> mapPtsDeg) {
		  
		  Iterator it = mapPtsDeg.entrySet().iterator();
		  
		  while (it.hasNext()) {
			  Map.Entry<Point, Integer> pair = (Map.Entry<Point, Integer>)it.next();
		      System.out.println(pair.getKey() + " = " + pair.getValue());
		  }
	  }
	
	
	 public Point pointDegreMax(LinkedHashMap<Point, Integer> mapPtsDeg, int edgeThreshold) {
		  
		  Iterator it = mapPtsDeg.entrySet().iterator();
		  	
	      Map.Entry<Point, Integer> pair = (Map.Entry<Point, Integer>)it.next();
	      
	      Point pointMaxVoisins = pair.getKey();
	      int max = Integer.valueOf(pair.getValue());
		  //System.out.println(pair.getKey() + " CSCSDVSDVC= " + pair.getValue());
		  
		  
		  while (it.hasNext()) {
			  Map.Entry<Point, Integer> pairSuite = (Map.Entry<Point, Integer>)it.next();
			  //System.out.println(pairSuite.getKey() + " C= " + pairSuite.getValue());
			 if(Integer.valueOf(pairSuite.getValue() ) > max ) {
				 max = Integer.valueOf(pairSuite.getValue() );
				 pointMaxVoisins = pairSuite.getKey();
			 }
				 
		  }
		  
		  //System.out.println(pointMaxVoisins + " DEG max = " + max);
		  
		  return pointMaxVoisins;
	  }
	 
	 public Point pointDegreMin(LinkedHashMap<Point, Integer> mapPtsDeg, int edgeThreshold) {
		  
		  Iterator it = mapPtsDeg.entrySet().iterator();
		  	
	      Map.Entry<Point, Integer> pair = (Map.Entry<Point, Integer>)it.next();
	      
	      Point pointMinVoisins = pair.getKey();
	      int min = Integer.valueOf(pair.getValue());
		  
		  while (it.hasNext()) {
			  Map.Entry<Point, Integer> pairSuite = (Map.Entry<Point, Integer>)it.next();
			 
			 if(Integer.valueOf(pairSuite.getValue() ) < min ) {
				 min = Integer.valueOf(pairSuite.getValue() );
				 pointMinVoisins = pairSuite.getKey();
			 }
				 
		  }
		  
		  //System.out.println(pointMinVoisins + " DEG min = " + min);
		  
		  return pointMinVoisins;
	  }
	
	
	 public LinkedHashMap<Point, Integer> cloneMap(LinkedHashMap<Point, Integer> assocPtsDeg) {
		  
		  LinkedHashMap<Point, Integer> copyMap = new LinkedHashMap<Point, Integer>();
		  
		  Iterator it = assocPtsDeg.entrySet().iterator();
		  
		  while (it.hasNext()) {
			  Map.Entry<Point, Integer> pair = (Map.Entry<Point, Integer>)it.next();
		      copyMap.put(new Point(pair.getKey()), Integer.valueOf(pair.getValue().intValue()) );
		  }
		  
		  return copyMap;
	  }
	 
	 
	 public ArrayList<Point> supprimerDoublons(ArrayList<Point> origins) {
		  
	      Set set = new HashSet() ;
	      set.addAll(origins) ;
	      ArrayList<Point> res = new ArrayList<>(set) ;	
	      //System.out.println("size list after supp doublons "+res.size());
	      return res;
			  
	  }
	 
	 
	 public ArrayList<Point> triCroissantDegPoint(LinkedHashMap<Point, Integer> assocPtsDeg, ArrayList<Point> origins, int edgeThreshold) {
		  
		  LinkedHashMap<Point, Integer> copyMap = cloneMap(assocPtsDeg);
		  
		  ArrayList<Point> originsWithoutDoublons = supprimerDoublons(origins);
		  ArrayList<Point> res = new ArrayList<>();
	  
		  while( res.size() != originsWithoutDoublons.size() ) {
		  
			  Point currentMin = pointDegreMin(copyMap, edgeThreshold);
		  
			  copyMap.remove(currentMin);
			  res.add(currentMin);
		  
			  //System.out.println("size map "+copyMap.size());		  
		  
		  }	  
	 
		  return res;
	  }
	 
	
	 public ArrayList<Point> triDecroissantDegPoint(LinkedHashMap<Point, Integer> assocPtsDeg, ArrayList<Point> origins, int edgeThreshold) {
	  	  
		  ArrayList<Point> tmp = triCroissantDegPoint(assocPtsDeg, origins, edgeThreshold);
		  
		  ArrayList<Point> res = new ArrayList<>();
		  
		  for(int j = tmp.size() - 1 ; j >= 0 ; j--) {
			  res.add(tmp.get(j));
			  //System.out.println("deg de "+tmp.get(j)+" : "+assocPtsDeg.get(tmp.get(j)) );
		  }
		  
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
	  
	  /*est voisin d'au moins 1 points de la liste*/
	  public boolean isVoisin(ArrayList<Point> origins,
			  				    Point toTest,
			  				    int edgeThreshold) {
		  
		  
		  for(Point p:origins)
			  if(toTest.distance(p) <= edgeThreshold ) {
				  System.out.println("le point testé est Voisin d'au moins un point de la liste");
				  return true;
			  }
			  
				  
		  
		  System.out.println("le point testé n'est pas voisin d'au moins un point de la liste");
		  return false;		  
		  
	  }
	  
	  
	  public boolean isDominant(ArrayList<Point> origins,
			  					ArrayList<Point> toTest,
			  					int edgeThreshold) {
		  
		  for(int i=0 ; i<toTest.size() ; i++) {
			  
				 Point p = toTest.get(i);
				 
				 if( origins.contains(p) )
					 continue;
				 else {
					 ArrayList<Point> voisinsP = e.neighbor(p, origins, edgeThreshold);
					 for(Point q : voisinsP) {
						 if(isVoisin(origins, q, edgeThreshold) )
							 continue;
						 else {
							 
							 System.out.println("is not dom");
							 return false;
						 }
							 
					 }
				 }
				 		  
		  }
		  System.out.println("is dom");
		  return true;
	  }
	  
	  
	  public ArrayList<Point> glouton(LinkedHashMap<Point, Integer> assocPtsDeg, 
							  		  ArrayList<Point> origins, 
							  		  ArrayList<Point> pointsTries, 
							  		  int edgeThreshold) {

				ArrayList<Point> tmp = cloneList(origins);  
				ArrayList<Point> res = cloneList(origins);
				
				//for(int i = 0 ; i < pointsTries.size() ; i++) {
				Random rand = new Random();
				
				for(int i=0 ; i<pointsTries.size() ; i++) {	
					Point toRemove = pointsTries.get(i);
					//System.out.println("point "+toRemove+" de deg "+assocPtsDeg.get(toRemove));
					tmp.remove(toRemove);	
					
					if(isDominant(origins, tmp, edgeThreshold) ) {
						//System.out.println("is valid");
					
						res.remove(toRemove);
					}
					else {
						//System.out.println("is not valid");
						//System.out.println("I "+i );
						tmp.add(toRemove);
					}
					
				}
				
				
				return res;
		}
	  
	 
	 
  public ArrayList<Point> calculDominatingSet(ArrayList<Point> points, int edgeThreshold) {
	  
	  points = supprimerDoublons(points);
	  ArrayList<Point> resGlouton; 
	  
	  LinkedHashMap<Point, Integer> mapDeg = initMapAssocPtsDeg(points, edgeThreshold);
	  
	  ArrayList<Point> pointsTriesCroissant = triCroissantDegPoint(mapDeg, points, edgeThreshold);
	  ArrayList<Point> pointsTriesDecroissant = triDecroissantDegPoint(mapDeg, points, edgeThreshold);

	  Point maxdeg = pointsTriesDecroissant.get(0);
	  
	  resGlouton = glouton(mapDeg, points, pointsTriesDecroissant, edgeThreshold);
	  
	  //afficherListeDegres(voisinsmaxdeg, mapDeg);
	  
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
