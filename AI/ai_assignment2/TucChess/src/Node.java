/**
 * Node class for minimax
 * @author gskoulas
 *
 */
public class Node {
	String move;
	int value;
	
	/**
	 * costrucor for node
	 * @param move 
	 * @param cost
	 */
	public Node(String move, int cost) {
		this.move = move;
		this.value = cost;
	}

}
