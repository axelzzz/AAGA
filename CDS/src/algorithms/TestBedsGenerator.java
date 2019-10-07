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
			
			for(int j = 0 ; j < nbLines ; j++) {
				Random r1 = new Random();
				int rand1 = r1.nextInt(800)+100; int rand2 = r1.nextInt(800)+100;
				
				files[i].write(rand1+" "+rand2+"\n");
			}			
			files[i].close();
		}
	}
	
	public static void main(String[] args) throws FileNotFoundException {
		
		TestBedsGenerator tbg = new TestBedsGenerator(5, 1000);		
		tbg.generate();
	}

}
