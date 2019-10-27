package algorithms;

import java.awt.Point;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class DefaultTeam {
	
	private static int cpt = 0;
	
	public boolean contains(ArrayList<Edge> edges, Point p1, Point p2) {		 
		for(Edge e:edges) {
			if((e.getP1()==p1 && e.getP2()==p2) || (e.getP2()==p1 && e.getP1()==p2)) {
				return true;
			}			
		}
		return false;
	}
	
	
	public boolean contains2(ArrayList<WEdge> wedges, Point p1, Point p2) {		 
		for(WEdge e:wedges) {
			if((e.getP1()==p1 && e.getP2()==p2) || (e.getP2()==p1 && e.getP1()==p2)) {
				return true;
			}			
		}
		return false;
	}
	
	
	//pour obtenir les aretes candidates en prenant en compte le seuil
	  public ArrayList<Edge> getTousEdges(ArrayList<Point> points, int edgeThreshold){ 
		  ArrayList<Edge> res = new ArrayList<Edge>();		 
		  for(int i = 0 ; i < points.size() ; i++){
			  Point pi = points.get(i);
			  for(int j = 0 ; j < points.size() ; j++){	
				  Point pj = points.get(j);
				  if(pi.equals(pj) || contains(res, pi, pj) )					  
					  continue;	
				  if(pi.distance(pj) <= edgeThreshold)
					  res.add(new Edge(pi, pj));				  		
			  }			  
		  }
		  System.out.println("ENDDDD");
		  return res;
	  }
	  
	  
	//calcule tableau de distances en fct de edgeThreshold
	public double[][] distances(ArrayList<Point> points, int edgeThreshold){
		
			double[][] res=new double[points.size()][points.size()];
			
			for(int i=0 ; i<points.size() ; i++){
				for(int j=0 ; j<points.size() ; j++){				
					Point pi = points.get(i);
					Point pj = points.get(j);				
					double dist = pi.distance(pj);
					if(dist < edgeThreshold)res[i][j]=dist;
					else{
						res[i][j]=Double.POSITIVE_INFINITY;
					}				
				}
			}			
			return res;
		}
		
		
	
	public ArrayList<WEdge> graphK(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints){  		  
		 
	      double[][] dist = distances(points, edgeThreshold);
		  ArrayList<WEdge> grapheK = new ArrayList<>();
		  
		  for(int i=0 ; i<hitPoints.size() ; i++){
			  for(int j=0 ; j<hitPoints.size() ; j++){
				  Point u = hitPoints.get(i);
				  Point v = hitPoints.get(j);
				  if(u.equals(v)) continue;					 
				  grapheK.add(new WEdge(u, v, dist[points.indexOf(u)][points.indexOf(v)]));				  
			  }
		  }		  
		  return grapheK;		  
	  }
	
		
	public ArrayList<Point> fromWEdgesToPoints(ArrayList<WEdge> wedges){		  
			 
		ArrayList<Point> res = new ArrayList<Point>();
			  
		for(WEdge e:wedges) {
			
			if(res.contains(e.getP1()) ){
				if(res.contains(e.getP2()) ) continue;
				res.add(e.getP2());				  
			}
			else {
				res.add(e.getP1());	
			}			  		  
		}
		
		return res;	  
	}
		 
		 
		
		
		
	public ArrayList<WEdge> trierWEdges(ArrayList<WEdge> wedges){  
			  
		ArrayList<WEdge> tmp = (ArrayList<WEdge>) wedges.clone();
			  
		ArrayList<WEdge> res = new ArrayList<WEdge>();		  
			  
		while(tmp.size()!=0) {
			
			double poidsMin = Double.MAX_VALUE;
			WEdge vectMin = null;
			
			for(WEdge ed:tmp){				  
				double poids = ed.getDistance();
				if (poidsMin > poids){
					poidsMin = poids;
					vectMin = ed;
				}				  
			}
			
			tmp.remove(vectMin);
			res.add(vectMin);	  			  
		}	
		
		return res;
	}
		
	
	
	public ArrayList<WEdge> trierWEdgesStream(ArrayList<WEdge> wedges){  
		
		List<WEdge> res = wedges.parallelStream()
								.filter(we -> we.getPoids() != Double.POSITIVE_INFINITY)
								.sorted(Comparator.comparing(WEdge::getPoids))								
								.collect(Collectors.toList());
		/*
		res.forEach(we -> {			
			System.out.println(we.getPoids()); 
		});
		*/
		
		return (ArrayList<WEdge>) res;
	}
	
	
		 
		 
		private Tree2D wedgesToTree(ArrayList<WEdge> wedges, Point root) { //pour exo 2
			    ArrayList<WEdge> remainder = new ArrayList<WEdge>();
			    ArrayList<Point> subTreeRoots = new ArrayList<Point>();
			    WEdge current;
			    //int i = 0;
			    while (wedges.size()!=0) {
			    	//System.out.println("i "+i);
			    	//i++;
			      current = wedges.remove(0);
			      //System.out.println("etiquette p1 : "+current.getP1().getEtiquette()+" etiquette p2 : "+current.getP2().getEtiquette());
			      if (current.getP1().equals(root)) {
			        subTreeRoots.add(current.getP2());
			      } else {
			        if (current.getP2().equals(root)) {
			          subTreeRoots.add(current.getP1());
			        } else {
			          remainder.add(current);
			        }
			      }
			    }
			    
			    ArrayList<Tree2D> subTrees = new ArrayList<Tree2D>();
			    for (Point subTreeRoot: subTreeRoots) subTrees.add(wedgesToTree((ArrayList<WEdge>)remainder.clone(),subTreeRoot));

			    return new Tree2D(root, subTrees);
		}
		 
		 
		 
		public Tree2D kruskal(ArrayList<WEdge> wedges) {		  
			  
			  	ArrayList<Point> points = fromWEdgesToPoints(wedges);	  	
				ArrayList<WEdge> aretes = (ArrayList<WEdge>) wedges.clone();
				
				ArrayList<WEdge> candidatesTriees = trierWEdges(aretes);
				
				ArrayList<WEdge> solution = new ArrayList<WEdge>();		
				
			    WEdge current;
			    NameTag forest = new NameTag(points);
			    
			    while (candidatesTriees.size()!=0) {
			    	
			      current = candidatesTriees.remove(0);
			      if (forest.tag(current.getP1())!=forest.tag(current.getP2())) {
			        solution.add(current);
			        forest.reTag(forest.tag(current.getP1()),forest.tag(current.getP2()));
			      }
			    }
			    return wedgesToTree(solution,solution.get(0).getP1());	    
		 	}
			
		
		
		public Tree2D kruskalStreamSort(ArrayList<WEdge> wedges) {		  
			  
		  	ArrayList<Point> points = fromWEdgesToPoints(wedges);	  	
			ArrayList<WEdge> aretes = (ArrayList<WEdge>) wedges.clone();
			
			ArrayList<WEdge> candidatesTriees = trierWEdgesStream(aretes);
						
			ArrayList<WEdge> solution = new ArrayList<WEdge>();		
			
		    WEdge current;
		    NameTag forest = new NameTag(points);
		    
		    while (candidatesTriees.size()!=0) {
		    	
		      current = candidatesTriees.remove(0);
		      if (forest.tag(current.getP1())!=forest.tag(current.getP2())) {
		        solution.add(current);
		        forest.reTag(forest.tag(current.getP1()),forest.tag(current.getP2()));
		      }
		    }
		    return wedgesToTree(solution,solution.get(0).getP1());	    
	 	}
		
		
		
		public ArrayList<WEdge> fromTreeToWEdges(Tree2D tree){ 		 

				 ArrayList<WEdge> res = new ArrayList<>();
				 Point root = tree.getRoot();
				 for(Tree2D t:tree.getSubTrees()) {
					 res.add(new WEdge(root, t.getRoot(), root.distance(t.getRoot())));
					 res.addAll(fromTreeToWEdges(t));
				 }
				 return res;
				
			  }
			
			
			
		public int[][] calculShortestPaths(ArrayList<Point> points, int edgeThreshold) {
			    int[][] paths= new int[points.size()][points.size()];
			    double[][] dist = distances(points, edgeThreshold);
			    
			    for(int i=0 ; i < paths.length ; i++){
			    	for(int j=0 ; j< paths.length ;j++){
			    		
			    		paths[i][j]=j; //dire qu'a l'init le plus court chemin de i a j est directement de i a j
			    	}
			    }	    
			    for (int k=0 ; k<paths.length ; k++){ 
			    	for (int i=0 ; i<paths.length ; i++){     		
			    		for (int j=0 ; j<paths.length ; j++){ 
			    			if(dist[i][j] > (dist[i][k] + dist[k][j]) ) {
			    				//System.out.println("DANS IF DE SHORTESTP");
			    				paths[i][j]=paths[i][k];
			    				//System.out.println("i : "+i+" j : "+j+" paths[i][j] : "+paths[i][j]);
			    				dist[i][j]=dist[i][k]+dist[j][k];	
			    				//System.out.println("i : "+i+" j : "+j+" dist[i][j] : "+dist[i][j]);
			    			}
			    		}
			    	}
			    	//System.out.println("k : "+k);
			    }
			    //return dist;
			    return paths;
			  }
			
			
		public ArrayList<WEdge> fromTreeToPathsOfG(Tree2D tree, ArrayList<Point> pointsDeG, int edgeThreshold){ 
			 ArrayList<WEdge> arbreDeK = fromTreeToWEdges(tree);
			 ArrayList<WEdge> graphH = new ArrayList<>();
			
			 int[][] t = calculShortestPaths(pointsDeG, edgeThreshold);	

			 for(WEdge we:arbreDeK) {
				Point from = we.getP1();
				Point to = we.getP2();				 
				Point next = pointsDeG.get(t[pointsDeG.indexOf(from)][pointsDeG.indexOf(to)]);
					 
				ArrayList<Point> cheminFromTo = new ArrayList<>();
				cheminFromTo.add(from);
				while(!from.equals(to)) {
					
					cheminFromTo.add(next);					 
					from = next;
					next = pointsDeG.get(t[pointsDeG.indexOf(from)][pointsDeG.indexOf(to)]);					 
				}
				
				cheminFromTo.add(to);
				//System.out.println("taille du chemin "+cheminFromTo.size());
					 
				for(int i=0 ; i<cheminFromTo.size() -1 ; i++) {
					Point pi = cheminFromTo.get(i);
					Point piPlusUn = cheminFromTo.get(i+1);
					graphH.add(new WEdge(pi, piPlusUn, pi.distance(piPlusUn)));
				}				 
			}	
			 
			 return graphH;
		 }	
		
	
		
		public Tree2D calculSteiner(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {
		    
			  //etape 1 init graphK
			  ArrayList<WEdge> grapheK = graphK(points, edgeThreshold, hitPoints);	

			  //etape 2 kruskal(K)
			  Tree2D t = kruskalStreamSort(grapheK);	  
			  
			  //etape 3 H = traduire T en chemins dans G	 
			  ArrayList<WEdge> grapheH = fromTreeToPathsOfG(t, points, edgeThreshold);
			  
			  //etape 4
			  Tree2D steinerTree = kruskalStreamSort(grapheH); 
			
			  return steinerTree;
		  }
		
	
		
		public ArrayList<Point> thePath(WEdge e,List<Point> noDoublon,int [][] pa){
			ArrayList<Point> res = new ArrayList<Point>();
			Point tmp = e.getP1();
			
	 		while(!tmp.equals(e.getP2())){
	 			res.add(tmp);
				int i = noDoublon.indexOf(tmp);
				int j = noDoublon.indexOf(e.getP2());
				tmp = noDoublon.get(pa[i][j]);
	 		}
	 		res.add(e.getP2());
	 		return res;
		}
		
		
		
		public ArrayList<Point> calculSteiner2(ArrayList<Point> hitPoints, ArrayList<Point> points, int edgeThreshold) {

			int [][] distPath = calculShortestPaths(points, edgeThreshold);
		
			ArrayList<WEdge> graphK = graphK(points, edgeThreshold, hitPoints);		
			
			ArrayList<WEdge> kruskalK = fromTreeToWEdges(kruskal(graphK));
			
			ArrayList<Point> steiner = new ArrayList<Point>();
			
			for(int i = 0 ; i < kruskalK.size() ; i++){
				WEdge e = kruskalK.get(i);
				ArrayList<Point> p = thePath(e,points,distPath);
				for(Point p1 : p)
					if(!steiner.contains(p1)) 
						steiner.add(p1);				
			}
			return steiner;
		}
		
		
		
		
		public ArrayList<Point> calculSteiner2StreamSort(ArrayList<Point> hitPoints, ArrayList<Point> points, int edgeThreshold) {

			int [][] distPath = calculShortestPaths(points, edgeThreshold);
		
			ArrayList<WEdge> graphK = graphK(points, edgeThreshold, hitPoints);		
			
			ArrayList<WEdge> kruskalK = fromTreeToWEdges(kruskalStreamSort(graphK));
			
			ArrayList<Point> steiner = new ArrayList<Point>();
			
			for(int i = 0 ; i < kruskalK.size() ; i++){
				WEdge e = kruskalK.get(i);
				ArrayList<Point> p = thePath(e,points,distPath);
				for(Point p1 : p)
					if(!steiner.contains(p1)) 
						steiner.add(p1);				
			}
			return steiner;
		}
		
		
		
		
		public boolean estVoisin(Point a, Point b,  int edgeThreshold) {
			
			if(a.distance(b)<=edgeThreshold)return true;
			return false;		
		}
		
		public boolean estVoisinDauMoinsN(Point a,  ArrayList<Point> points, int edgeThreshold, int n) {
			
			int cpt = 0;
			for(Point p:points) {
				if(a.equals(p))continue;
				if(estVoisin(a, p, edgeThreshold))cpt++;
			}
			if(cpt >= n)return true;
			return false;		
		}
		
		public boolean estVoisinDePersonne(Point a, ArrayList<Point> points, int edgeThreshold) {
			for(int i=0 ; i<points.size() ; i++) {
				Point p = points.get(i);
				if(a.distance(p) <= edgeThreshold)return false;
			}
			return true;
		}
		
		
		public boolean estVoisinDePersonneVStream(Point a, ArrayList<Point> points, int edgeThreshold) {
			
			Stream<Point> sp = points.stream();			
			return sp.allMatch(p -> a.distance(p) > edgeThreshold);
		}
		
		
		/*
		public ArrayList<Point> getStable1(ArrayList<Point> points, int edgeThreshold){
			
			ArrayList<Point> res = new ArrayList<>();		
			
			res.add(points.get(0));
			for(Point p:points) {			
				if(estVoisinDePersonne(p, res, edgeThreshold))res.add(p);		
			}		
			return res;		
		}
		
		
		
		
		public ArrayList<Point> stable2(ArrayList<Point> points, int edgeThreshold){
			Random r = new Random();
			ArrayList<Point> res = new ArrayList<>();
			int rand = r.nextInt(points.size());
			//System.out.println("R "+rand);
			res.add(points.get(rand));
			for(int i=0 ; i<points.size() ; i++) {	
				Point p = points.get(i);
				if(estVoisinDePersonne(p, res, edgeThreshold))res.add(p);		
			}		
			return res;
		}
		
		
		
		public ArrayList<Point> getStable2(ArrayList<Point> points, int edgeThreshold){
			ArrayList<Point> stable = stable2(points, edgeThreshold);
			int cpt=0;
			while(cpt < 100 && points.size() > 0) {
				//System.out.println("nb pts "+points.size());
				ArrayList<Point> tmp = stable2(points, edgeThreshold);
				if(tmp.size() > stable.size())stable = tmp;
				cpt++;
			}
			return stable;		
		}
		*/
		
		
		public ArrayList<Point> stable2VStream(ArrayList<Point> points, int edgeThreshold){
			
			Random r = new Random();
			ArrayList<Point> res = new ArrayList<>();
			int rand = r.nextInt(points.size());
			res.add(points.get(rand));			
			
			Stream<Point> sp = points.stream();			
			sp.forEach(p -> {
				if(estVoisinDePersonneVStream(p, res, edgeThreshold))res.add(p);		
			});
										
			return res;
		}
		
		
		
				
		public ArrayList<Point> getStable2VStream(ArrayList<Point> points, int edgeThreshold){
			
			ArrayList<Point> stable = stable2VStream(points, edgeThreshold);
			
			int cpt=0;
			while(cpt < 100) {
				//System.out.println("nb pts "+points.size());
				ArrayList<Point> tmp = stable2VStream(points, edgeThreshold);
				if(tmp.size() < stable.size())stable = tmp;
				cpt++;
			}
			return stable;		
		}
		
		
		
		public HashMap<Point, Color> marquageInit(ArrayList<Point> udg, ArrayList<Point> mis){
			
			HashMap<Point, Color> res = new HashMap<>();		
			for(Point p:udg) {			
				if(mis.contains(p)) 
					res.put(p, Color.BLACK);			
				else 
					res.put(p, Color.GREY);				
			}		
			return res;
		}
		
		
		
		
		public ArrayList<Point> getBlackNodes(HashMap<Point, Color> map){
			ArrayList<Point> blackNodes = new ArrayList<>();
			
			for(Entry<Point, Color> entry : map.entrySet()) {
				if(entry.getValue() == Color.BLACK)blackNodes.add(entry.getKey());
			}
			return blackNodes;    
		}
		
		
		
		
		public int nbBlueNodes(HashMap<Point, Color> map) {
			int sum=0;
			for(Entry<Point, Color> entry : map.entrySet())if(entry.getValue() == Color.BLUE)sum++;
			return sum;
		}
		
		
		
		
		public int nbGreyNodes(HashMap<Point, Color> map) {
			int sum=0;
			for(Entry<Point, Color> entry : map.entrySet())if(entry.getValue() == Color.GREY)sum++;
			return sum;
		}
		
		
		
				
		//quand on cherche si le point en parametre (qui est normalement gris) est voisin d'au moins i black nodes
		public boolean estVoisinDeAtLeastIblackNodesDeDifferentsBBComp(HashMap<Point, Color> map,  Point p, int n, ArrayList<ArrayList<Point>> comps, int edgeThreshold) {		
					
			int tmp = 0;
					
			for(int i=0 ; i<comps.size() ; i++) {
				ArrayList<Point> comp = comps.get(i);		
				if(estVoisinPointComp(comp, p, edgeThreshold, map)){
					tmp++;
					if(tmp >= n)return true;
				}
			}	
			return false;		
		}
				
			
		
		
		
		//recuperer les noeuds noirs d'un composant
		public ArrayList<Point> getBlackNodesFromTheBBComp(HashMap<Point, Color> map, ArrayList<Point> comp) {
					
			ArrayList<Point> res = new ArrayList<Point>();
					
			for(Entry<Point, Color> entry : map.entrySet()) {
				Point p = entry.getKey();
				//on parcourt les dominateurs, si ils font partie du composant on les retourne
				if(entry.getValue() == Color.BLACK) {				
					if(comp.contains(p)) res.add(p);				
				}
			}
			return res;			
		}	
				
				
				
		public boolean estVoisinPointComp(ArrayList<Point> comp, Point p, int edgeThreshold, HashMap<Point, Color> map){
			
			//si le point est voisin d'un blacknode du composant, alors il est voisin du composant
			ArrayList<Point> blacknodes = getBlackNodesFromTheBBComp(map, comp);
			
			for(int i=0 ; i<blacknodes.size() ; i++) {
				Point b = blacknodes.get(i);
				if(b.distance(p) <= edgeThreshold) 					
					return true;
				
			}
			return false;
		}



			
		public ArrayList<ArrayList<Point>> getCompVoisinsOfPoint(Point p, HashMap<Point, Color> map, int edgeThreshold, ArrayList<ArrayList<Point>> comps){
				
			ArrayList<ArrayList<Point>> voisins = new ArrayList<>();
				
			for(int i=0 ; i<comps.size() ; i++) {
				ArrayList<Point> comp = comps.get(i);
				if(estVoisinPointComp(comp, p, edgeThreshold, map))voisins.add(comp);
			}
				
			return voisins;		
		}

		
		
		public boolean existGreyNodeAdjToAtLeastIBlackNodeInDifferentBBComp(HashMap<Point, Color> map, int edgeThreshold, ArrayList<ArrayList<Point>> comps, int n) {
			
			boolean exist = false;
						
			for(Entry<Point, Color> entry : map.entrySet()) {
				
				cpt++;
				Point p = entry.getKey();
				Color c = entry.getValue();			
									
				//si le point courant gris est voisin de n noeuds noirs de differents black/blue components, on le transforme en bleu, 
				//on fusionne les composants voisins et on ajoute le noeud courant au resultat de la fusion
				if(c == Color.GREY && estVoisinDeAtLeastIblackNodesDeDifferentsBBComp(map, p, n, comps, edgeThreshold)) {
					//le point gris devient un bleu
					entry.setValue(Color.BLUE);	
					ArrayList<ArrayList<Point>> compsVoisins = getCompVoisinsOfPoint(p, map, edgeThreshold, comps);
					//on supprime les comps voisins de la liste des comps,  
					comps.removeAll(compsVoisins);
					ArrayList<Point> fusion = new ArrayList<>();
					//on les fusionne
					for(ArrayList<Point> compsv : compsVoisins) {
						fusion.addAll(compsv);
					}
					//puis ajoute le point,
					fusion.add(p);
					// puis le resultat de tout ca est reinjecte dans la liste des composants
					comps.add(fusion);
						
					exist = true;
				}
					
					
				
				//System.out.println("cpt : "+cpt);
				
			}
		
			return exist;
		}
		
		
		
		public ArrayList<Point> AlgoA(HashMap<Point, Color> map, int edgeThreshold){
			ArrayList<Point> res = new ArrayList<>();
			ArrayList<Point> dominants = getBlackNodes(map);
			ArrayList<ArrayList<Point>> components = new ArrayList<>();		
			
			

			//au depart chaque noeud de l'ensemble dominant est considere comme un composant bleu/noir connexe
			for(int i = 0 ; i < dominants.size() ; i++){
				
				components.add(new ArrayList<>());
				components.get(i).add(dominants.get(i));
			}			
			
			
	
			for(int i = 5 ; i >= 2 ; i--) {

				
				
				while(existGreyNodeAdjToAtLeastIBlackNodeInDifferentBBComp(map, edgeThreshold, components, i)) {
					//System.out.println("nb blue nodes "+nbBlueNodes(map));
					//System.out.println("nb grey nodes "+nbGreyNodes(map));
					
				}
				//System.out.println("cpt : "+cpt);
				
			}
					
			for(Entry<Point, Color> entry : map.entrySet())if(entry.getValue() == Color.BLUE)res.add(entry.getKey());
			//System.out.println("nb blue nodes "+nbBlueNodes(map));
			res.addAll(dominants);
			
			return res;
		}
		
		
		
		//on parcourt la liste des points, pour chaque point on cree une liste de ses voisins
		//si chaque point est voisin d'un autre alors tous les points sont atteignables, d'ou ils sont connexes
		public boolean estConnexe(ArrayList<Point> points, int edgeThreshold) {
			
			ArrayList<Point> toVisit = new ArrayList<>();
			ArrayList<Point> notVisited = new ArrayList<>(points);
			
			toVisit.add(notVisited.remove(0));
			
			while(toVisit.size() > 0) {
				
				Point tmp = toVisit.remove(0);
				ArrayList<Point> tmpL = new ArrayList<>();
				
				for(Point p : notVisited) {
					if(tmp.distance(p) <= edgeThreshold)tmpL.add(p);
				}
				
				//toVisit.remove(tmp);			
				toVisit.addAll(tmpL);
				notVisited.removeAll(tmpL);
				//System.out.println("dans while");
			}
			
			if(notVisited.size() == 0)return true;
			return false;		
			
		}	
		
		
		
		
		//chaque point de l'ensemble de depart est soit dans toTest soit un voisin d'au moins un point de toTest
		public boolean estStable(ArrayList<Point> points, ArrayList<Point> toTest, int edgeThreshold){
	        int cpt=0;

	        for(Point p: points){
	            for(Point q : toTest){
	                if( !p.equals(q) && p.distance(q) < edgeThreshold)
	                    cpt ++;
	            }
	            if(cpt == 0)
	                return false;
	            cpt =0;
	        }

	        return true;
	    }
		
		
		
		
		//heuristique de suppression
		//on parcourt la liste, si le point courant n'est pas un dominant et que si le graphe reste connexe en supprimant le point, on le supprime sinon on passe au suivant
		public ArrayList<Point> heuristique1(ArrayList<Point> tree, ArrayList<Point> points, int edgeThreshold){
			
			ArrayList<Point> res = new ArrayList<Point>(tree);
			
			//parcours dans l'ordre de la liste
			for(Point p:tree) {
				//System.out.println("ICI");
				ArrayList<Point> tmp = new ArrayList<>(tree);
					
				tmp.remove(p);
				//si le graphe avec suppression du point courant est connexe et tjr un ens dominant, on maj le graphe
				if(estConnexe(tmp, edgeThreshold) && isDominant(points, tmp, edgeThreshold)) {
					//System.out.println("estCONNEXE");
					res = tmp;				
				}						
			}
			
			return res;		
		}
		
		
		
		public ArrayList<Point> heuristique1AvecBoucle(ArrayList<Point> tree,ArrayList<Point> points, int edgeThreshold, int n){
			int i = 0;
			int nbloopMaxUseful = 0;
			while(i < n) {
				ArrayList<Point> tmp = heuristique1(tree, points, edgeThreshold);
				if(tmp.size() < tree.size()) {
					tree = tmp;
					nbloopMaxUseful++;
				}
				i++;
			}
			System.out.println(nbloopMaxUseful);
			return tree;
		}
		
		
		
		
		public ArrayList<Point> rest(ArrayList<Point> points, ArrayList<Point> toRemove) {
			  
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
		  
		  
		  
		
		public ArrayList<Point> neighbor(Point p, ArrayList<Point> vertices, int edgeThreshold){
			    ArrayList<Point> result = new ArrayList<Point>();
	
			    for (Point point:vertices) if (point.distance(p)<edgeThreshold && !point.equals(p)) result.add((Point)point.clone());
	
			    return result;
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
		
		
		
		
		//on parcourt le graphe, si on peut remplacer 2 points par 1, on le fait
		public ArrayList<Point> localSearchV1(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
			
			ArrayList<Point> copyReste = cloneList(reste);
			 
			ArrayList<Point> tmp = cloneList(solution);	  
			
			Random rand = new Random();
			  
			Point p = tmp.remove(rand.nextInt(tmp.size()));
			Point q = tmp.remove(rand.nextInt(tmp.size()));
			  
			  
			  
			Point r = copyReste.remove(rand.nextInt(reste.size()));
			  
			tmp.add(r);
			  
			//remplacer estStable par estDominant
			if( estConnexe(tmp, edgeThreshold) && isDominant(origins, tmp, edgeThreshold) ) {
				System.out.println("is valid");
				return tmp;
			}
			else {
				//System.out.println("is not valid");
				return solution;
			}
				  
		}
		
		
		
		
		  public ArrayList<Point> localSearchV12(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
			  
			  Random rand = new Random();
		
			  Point p = solution.remove(rand.nextInt(solution.size()));
			  Point q = solution.remove(rand.nextInt(solution.size()));
			  
			  Point r = reste.remove(rand.nextInt(reste.size()));
			  
			  solution.add(r);
			  
			  if ( estConnexe(solution, edgeThreshold) && isDominant(origins, solution, edgeThreshold) ) {
				  System.out.println("is valid, new size : "+solution.size());
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
		  
		  
		  
		
		//on parcourt le graphe, si on peut remplacer 2 points par 1, on le fait
		public ArrayList<Point> loopLocalSearchV1(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold, int n) {
					
			ArrayList<Point> res = new ArrayList<>();
			
			for(int i=0 ; i<n ; i++) 
				res = localSearchV12(origins, solution, reste, edgeThreshold);
			
			return res;
		}
		
		
		
			
			
				
		
		public ArrayList<Point> permutation(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold) {
			
			  Random rand = new Random();
			  
			  Point p = solution.remove(rand.nextInt(solution.size()));			  
			  Point q = reste.remove(rand.nextInt(reste.size()));
			  
			  solution.add(q);			  
			  
			  if( estConnexe(solution, edgeThreshold) && isDominant(origins, solution, edgeThreshold) ) {
				  //System.out.println("is valid");
				  return solution;
			  }
			  else {
				  //System.out.println("is not valid");
				  solution.remove(q);
				  solution.add(p);
				  reste.add(q);
				  //return tmp;
				  return solution;
			  }
				  
		  }
		
		
		
		
		public ArrayList<Point> loopLocalSearchPermut(ArrayList<Point> origins, ArrayList<Point> solution, ArrayList<Point> reste, int edgeThreshold, int n) {
			
			ArrayList<Point> res = localSearchV12(origins, solution, reste, edgeThreshold);
				
			for(int i=0 ; i<n ; i++) {
				res = localSearchV12(origins, solution, rest(origins, res), edgeThreshold);
				res = permutation(origins, res, rest(origins, res), edgeThreshold);
			}
				
			return res;
		}
		
		
		
		
  public ArrayList<Point> calculConnectedDominatingSet(ArrayList<Point> points, int edgeThreshold) {
	//etape 1 calcul MIS, (on peut tenter de maximiser sa taille avec getStable2)
	 //Tree2D steinerT = calculSteiner(points, edgeThreshold, points);
	 //ArrayList<Point> stable = fromWEdgesToPoints(fromTreeToWEdges(steinerT));
	  //ArrayList<Point> stable = stable2(points, edgeThreshold);
	  
	  
	  
	 ArrayList<Point> stable = getStable2VStream(points, edgeThreshold);	 
	
		
	//etape 2 marquage
	 
	
		HashMap<Point, Color> mark = marquageInit(points, stable);			
		 
		
	//etape 3 calcul CDS	
		//ArrayList<Point> result = calculSteiner2(stable, points, edgeThreshold);;	
		//ArrayList<Point> result = calculSteiner2StreamSort(points, points, edgeThreshold);
		ArrayList<Point> result = AlgoA(mark, edgeThreshold);
		
		//ArrayList<WEdge> graphK = graphK(points, edgeThreshold, points);	
		//ArrayList<WEdge> kruskalK = fromTreeToWEdges(kruskalStreamSort(graphK));
		//ArrayList<Point> result = fromWEdgesToPoints(kruskalK);
		
		
		
		
		
		//ArrayList<WEdge> gK = graphK(points, edgeThreshold, points);
		
		//long startTime1 = System.nanoTime();
		
		//trierWEdgesStream(gK);
		/*
		Tree2D couvrant = calculSteiner(points, edgeThreshold, points);
		
		long endTime1 = System.nanoTime();
		long duration = endTime1-startTime1;
		duration = duration / 1000000;
		System.out.println("Time function : "+duration+"ms");
				
		
		ArrayList<Point> result = fromWEdgesToPoints(fromTreeToWEdges(couvrant));
		*/
		/*
		ArrayList<Point> sub = new ArrayList<>(points);
		sub.removeAll(result);
		System.out.println("here "+points.indexOf(sub.get(0)));
		*/
		
		//result.add(points.get(978));
		
		//result = heuristique1AvecBoucle(result, points, edgeThreshold, 200);
		return result;
		
		//return loopLocalSearchV1(points, result, reste(points, result), edgeThreshold, 1000000);
		//return loopLocalSearchPermut(points, result, rest(points, result), edgeThreshold, 100000);
		
		//return stable;
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
