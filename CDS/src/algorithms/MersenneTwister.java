package algorithms;

public class MersenneTwister {
	
	private int[] mt;
	private int index;
	
	public MersenneTwister() {
		
		mt = new int[624];
		index = 0;
	}
	
	
	
	public void initialize_generator(int seed) {
		
		index = 0;
		mt[0] = seed;
		
		for(int i=1 ; i<=623 ; i++) {
			long myLong = ( 1812433253 * ( mt[i-1]^( mt[i-1]>>>30 ) ) +i );
			long tmp = ((myLong >> 32) << 32); //shift right then left
			mt[i]=(int)(myLong - tmp);
		}
	}

	
	
	public int extract_number() {
		
		if(index == 0)
			generate_numbers();
		
		int y = mt[index];
		y = y^(y>>>11);
		y = (int) (y^( (y<<7) & Long.valueOf("2636928640").longValue()));
		y = (int) (y^( (y<<15) & Long.valueOf("4022730752").longValue()));
		y = y^(y>>>18);
		
		index = (index+1) % 624;
		return y;
	}
	
	
	public void generate_numbers() {
		
		for(int i=0 ; i <= 623 ; i++) {
			
			long y = (mt[i] & 0x80000000)
					+ (mt[(i+1)%624] & 0x7fffffff);
			
			mt[i] = (int) (mt[(i+397)%624]^(y>>>1));
			
			if(y%2 != 0)
				mt[i] = (int) (mt[i]^Long.valueOf("2567483615").longValue());
		}
	}
	
}
