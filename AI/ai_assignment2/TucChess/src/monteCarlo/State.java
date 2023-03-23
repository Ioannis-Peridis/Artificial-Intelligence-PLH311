package monteCarlo;

import java.util.ArrayList;
/**
 * State class 
 * Contains all the needed value for each node
 * @author gskoulas
 *
 */
public class State {
	
	String[][] board;
	int color;
	int visitCount;
	int cost;
	String move;
	int scoreBlack;
	int scoreWhite;
	
	/**
	 * constructor for State class
	 * @param board the board of the node
	 * @param color color of player 
	 * @param move the move that got us to this situation
	 */
	public State(String [][] board, int color, String move) {
		this.move = move;
		this.board = board;
		this.color = color;
		visitCount = 0;
		cost = 0;
		scoreBlack = 0;
		scoreWhite = 0;
	}
	
	/**
	 * function for adding 1 to visitedCount
	 */
	public void incrementVisitCount() {
		this.visitCount++;
	}
	
//	public ArrayList<State> getAvailableStates(){
//		
//	}
//	
//	public void randomPlay() {
//		
//	}

}
