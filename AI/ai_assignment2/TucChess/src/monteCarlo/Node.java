package monteCarlo;

import java.util.ArrayList;
/**
 * Node class
 * @author gskoulas
 *
 */
public class Node {
	State state;
	Node parent;
	ArrayList<Node> children;
	
	/**
	 * costructor of node
	 * @param parent parent node
	 * @param state state that contains all the needed values for node
	 */
	public Node(Node parent, State state) {
		children =new ArrayList<>(); 
		this.state = state;
		this.parent = parent;
	}
	
	/**
	 * function checking if node is leaf
	 * @return true if node is leaf. False in any other case
	 */
	public boolean isLeaf() {
		if (children.isEmpty()) 
			return true;
		return false;
			
	}

}