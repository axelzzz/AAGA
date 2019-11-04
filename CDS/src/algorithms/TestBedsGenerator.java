package algorithms;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Random;

public class TestBedsGenerator {
	
	private PrintWriter[] files;	
	private int nbLines;
	
	public TestBedsGenerator(int nbFiles, int nbLines) throws FileNotFoundException {		
		files = new PrintWriter[nbFiles];
		this.nbLines = nbLines;		
	}
	
	public void generate() throws FileNotFoundException {
		
		for(int i = 0 ; i < files.length ; i++) {
			
			files[i] = new PrintWriter("input"+i+".points");			
			int j=0;
			
			while(j<nbLines) {
				
				Random r1 = new Random();
				int rand1 = r1.nextInt(800)+100; int rand2 = r1.nextInt(700)+100;
				
				if( (rand1 > 150 && rand1 < 850 && rand2 > 100) 
				 || (rand2 > 150 && rand2 < 750 && rand1 > 100) ) {
					files[i].write(rand1+" "+rand2+"\n");
					j++;
				}				
			}			
			
			files[i].close();
		}
	}
	
	
	
	public void generateMersenneTwister() throws FileNotFoundException {
		
		MersenneTwister mt = new MersenneTwister();
		mt.initialize_generator(666);
		
		for(int i = 0 ; i < files.length ; i++) {
			
			files[i] = new PrintWriter("input"+i+".points");
			
			for(int j=0 ; j<nbLines ; j++) {
				
				int x = mt.extract_number()/10000000;			
				while(x<=0)
					x = mt.extract_number()/10000000;
				
				int y = mt.extract_number()/10000000;
				while(y<=0)
					y = mt.extract_number()/10000000;
				

				x=x*3;
				y=y*3;
					
				
				System.out.println("x : "+x+" y : "+y);
				files[i].write(x+" "+y+"\n");
			}	
			
			files[i].close();
		}
	}
	
	public static void main(String[] args) throws FileNotFoundException {
		
		TestBedsGenerator tbg = new TestBedsGenerator(1, 1000);		
		//tbg.generate();
		tbg.generateMersenneTwister();
	}

}
