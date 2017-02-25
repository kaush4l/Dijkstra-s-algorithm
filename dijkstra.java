import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

public class dijkstra {

	public String source;
	public String dest;
	private static Map<String,Vertex> vertexMap = new HashMap<String,Vertex>( );
	
	public dijkstra(String source, String dest, Map<String, Vertex> v) {
		this.source=source;
		this.dest= dest;
		vertexMap=v;
	}
	 private void printPath( Vertex dest )
	    {
	        if( dest.prev != null )
	        {
	            printPath( dest.prev );
	            System.out.print( " to " );
	        }
	        System.out.print( dest.name );
	    }
	void shortestpath(  ){
		for( Entry<String, Vertex> entry : vertexMap.entrySet( ) ){
    		Vertex v = entry.getValue();
    		v.reset();
		}
		Vertex v = vertexMap.get(source);
		v.dist = 0;
		
		int ArraySize = 0;
		for( Entry<String, Vertex> entry : vertexMap.entrySet( ) ){
    		Vertex w = entry.getValue( );
    		if( w.status == true )
    			ArraySize++;
		}
		Vertex[] V = new Vertex[ ArraySize + 1 ];
		int i = 1;
		for( Entry<String, Vertex> entry : vertexMap.entrySet( )){
    		Vertex w = entry.getValue();
    		if(w.status == true){
    		V[i] = w;
    		i++;
    		}
		}
		Bulid_Min_Heap( V , ArraySize );
		while( true ) {
		v = Heap_Extract_Min( V, ArraySize );
		if( v != null ) {
			ArraySize--;
			for( Edge e : v.adj) {
				if(e.status == true) {
				String[] split = e.name.split("_");
    			String secoundVertex = split[1];
    			Vertex w = vertexMap.get(secoundVertex);
    			if(w.status == true) {
    				if(w.dist>e.weight+v.dist) {
    					w.dist= e.weight + v.dist;
//    					System.out.println(w.dist + " = " + v.dist + " + " + e.weight);
    					w.prev=v;
    					}
    				}
    			}
				
			}
			
			if( ArraySize > 0 )
			Bulid_Min_Heap ( V, ArraySize );
			else {
//				System.out.println("empty Que");
				break;
			}
		}
		
		
		
//		System.out.println(v.name);
	}

	Vertex z = vertexMap.get(dest);
	System.out.println( z.name + " " + z.prev.name + z.dist );
	System.out.println();
	printPath(z);
	}

	public void Bulid_Min_Heap(Vertex[] A, int ArraySize) {
		
		for( int i = ArraySize/2; i >= 1; i--) {
			Min_Heapify(A,i,ArraySize);
		}
	}

	public void Min_Heapify(Vertex[] a, int i, int n) {
		Vertex[] A = a;
		int l =( 2 * i );
		int r = l + 1;
		int min = i;
		if( l <= n ){
			if( A[l].dist<A[i].dist ){
			min = l;
			}
		}
		if(r<=n  ){
			if( A[r].dist < A[min].dist){
			min = r;
			}
		}
		if( min != i ){
			
			Vertex x = A[i];
			A[i]=A[min];
			A[min] = x;
			Min_Heapify(A,min,n);
		}
	}
	public Vertex Heap_Extract_Min(Vertex[] A, int n){
		if(n<1){
//			System.out.println("Que is empty");
		return null;
		}
		Vertex min=A[1];
		A[1]=A[n];
		return min;
	}
}
