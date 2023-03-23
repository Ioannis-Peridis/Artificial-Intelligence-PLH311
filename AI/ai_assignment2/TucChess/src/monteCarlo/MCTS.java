package monteCarlo;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Random;

/**
 * Monte carlo search tree
 * @author gskoulas
 *
 */
public class MCTS {

	private int rookBlocks = 3;		// rook can move towards <rookBlocks> blocks in any vertical or horizontal direction
	int totalVisits;	// total visits of the nodes
	World game;		// the world
	public static int MAX = 1000000;	// big int used for initialization
	String [][] tmpBoard;
	int rows;
	int columns;
	int scoreBlack;
	int scoreWhite;
	int it = 0;

	public MCTS(World game) {
		totalVisits = 0;
		this.game = game;
		rows = game.getRows();
		columns = game.getColumns();
		scoreWhite = 0;
		scoreBlack = 0;
	}

	/**
	 * function for changing color 
	 * @param color current color
	 * @return black(1) if given as input white(0) and 0 if given 1
	 */
	public int changeColor(int color) {
		if (color == 0) 
			return 1;
		return 0;
	}

	/**
	 * function that gets all the available moves for the black or the white player
	 * @param board the board
	 * @param color
	 * @return
	 */	public ArrayList<String> getAvailableMoves(String[][] board, int color){
		if (color == 0) 
			return this.whiteMoves(board);
		return this.blackMoves(board);
	}

	 /**
	  * start function of monte carlo algorithm
	  * @param board 
	  * @param color
	  * @return the move that is gonna be executed
	  */
	public String findNextMoveMonteCarlo(String [][] board, int color) {
		totalVisits =0;
		tmpBoard = equalBoards(game.getBoard());
		String bestMove = "";
		int eval = -MAX; 
		Tree tree = new Tree(new Node(null, new State(board, 0,null)));
		expand(tree.root,color);
		monteCarlo(board, color, tree);
		for (Node child : tree.root.children) {
			if(child.state.cost > eval) {
				bestMove = child.state.move;
				eval = child.state.cost;
			}
		}

		System.out.println("Best move:"+bestMove);
		return bestMove;
	}

	/**
	 * main function for monte carlo. It executes all the steps needed
	 * @param board
	 * @param color
	 * @param tree
	 */
	public void monteCarlo(String[][] board, int color,Tree tree) {
		double start = System.currentTimeMillis();
		while(System.currentTimeMillis()-start < 2000) {
			scoreBlack = game.scoreBlack;
			scoreWhite = game.scoreWhite;
			Node leaf = selection(tree.root, color);
			Node newleaf = expand(leaf, leaf.state.color);
			double evaluation = rollout(newleaf);
			backPropagation(newleaf, evaluation);
			//System.out.println(evaluation);
			totalVisits++;

		}

	}

	/**
	 * function that selects the next leaf node
	 * @param root of the tree xearching
	 * @param color of player
	 * @return leaf node
	 */
	public Node selection(Node root, int color) {
		Node selectedNode = root;
		int i = 0;
		while (!selectedNode.isLeaf()) {
			double best = Double.NEGATIVE_INFINITY; 

			for(Node node : selectedNode.children) {

				if (best < uctValue(node.state.visitCount, node.state.cost, totalVisits)) {
					best = uctValue(node.state.visitCount, node.state.cost, totalVisits);
					selectedNode  = node;
				}
			}
		}
		return selectedNode;
	}

	
	/**
	 * function that expands a leaf node
	 * @param nodeToExpand the node that is going to be expanded
	 * @param color 
	 * @return the expanded node in which we continue
	 */
	public Node expand(Node nodeToExpand, int color) {
		ArrayList<String> moves = getAvailableMoves(tmpBoard, color);
		String [][] oldBoard = equalBoards(tmpBoard);
		if (!gameOver() && !moves.isEmpty()) {
			for(String move : moves) {
				int oldScoreBlack = scoreBlack;
				int oldScoreWhite = scoreWhite;
				makeMove(move, color);
				nodeToExpand.children.add(new Node(nodeToExpand, new State(tmpBoard, changeColor(color), move)));
				undoMove(oldBoard,oldScoreWhite, oldScoreBlack);
			}
		}
		else {
			tmpBoard = equalBoards(oldBoard);
			return nodeToExpand;
		}
		return nodeToExpand.children.get(randomNumberGenerator(0, nodeToExpand.children.size()-1));
	}


	/**
	 * rollout function that simulates a random play until we have game over
	 * @param node node to start
	 * @return the evaluation cost of the terminal node
	 */
	public double rollout(Node node) {
		Node tmp = node;
		double evaluation = 0;
		//String oldBoard[][] = equalBoards(tmpBoard);
		while(!gameOver()) {
			ArrayList<String> moves = getAvailableMoves(tmp.state.board, tmp.state.color);
			if (moves.isEmpty()) {
				break;
			}
			String move = simulateRandomPlay(moves);
			try {
				makeMove(move, tmp.state.color);
			} catch (Exception e) {
				//System.out.println("1");
			}
			tmp = new Node(null, new State(tmpBoard, changeColor(tmp.state.color), move));
		}

		evaluation = evaluateMCTS(tmp.state.color);
		tmpBoard = equalBoards(node.state.board);
		return evaluation;
	}

	/**
	 * better evaluate function for MCTS
	 * @param color
	 * @return the cost of the node
	 */
	public int evaluateMCTS(int color) {
		int eval = evaluateScore(tmpBoard, color);
		if (scoreWhite - scoreBlack == 0) 
			return 0;
		if(game.getMyColor() == 0)
			if (scoreWhite-scoreBlack > 0) 
				return 1;
			else
				return -3;
		else
			if(scoreBlack-scoreWhite > 0)
				return 1;
			else
				return -3;
	}

	/**
	 * function to update the values needed from all the visited nodes 
	 * @param leaf node from where we start the back propagation
	 * @param evaluation value added in each visited node
	 */
	public void backPropagation(Node leaf, double evaluation) {
		Node tmp = leaf;
		while(tmp != null) {
			tmp.state.cost += evaluation;
			tmp.state.incrementVisitCount();
			tmp = tmp.parent;
		}
	}

	/**
	 * function that simulates the random play
	 * @param moves available moves
	 * @return the move we are executing
	 */
	public String simulateRandomPlay(ArrayList<String> moves) {
		Random random = new Random();
		return moves.get(random.nextInt(moves.size()));
	}

	/**
	 * function that calculates the uctValue
	 * @param visitCount counter for node that shows how many times he is being visited
	 * @param score the evaluation
	 * @param totalVisit total number of visits
	 * @return the result of typou
	 */
	public double uctValue(int visitCount, int score, int totalVisit) {
		if (visitCount == 0) 
			return MAX;
		return ((double)score/(double)visitCount) + 1.41*Math.sqrt(Math.log(totalVisit/(double)visitCount));
	}


	/**
	 *  Function that executes a move to tmpBoard
	 * @param move is string that represents from which coordinates we are going to take a apwn and move it to its new coordinates
	 * @param color of the player moving a pawn
	 */
	public void makeMove(String move, int color) {
		int x1 = Integer.parseInt(move.substring(0, 1));
		int y1 = Integer.parseInt(move.substring(1, 2));
		int x2 = Integer.parseInt(move.substring(2, 3));
		int y2 = Integer.parseInt(move.substring(3, 4));
		String chesspart = "";

		chesspart = Character.toString(tmpBoard[x1][y1].charAt(1));

		boolean pawnLastRow = false;

		// check if it is a move that has made a move to the last line
		if(chesspart.equals("P"))
			if( ((x1==rows-2 && x2==rows-1) && color == 0) || ((x1==1 && x2==0) && color == 1) )
			{
				tmpBoard[x2][y2] = " ";	// in a case an opponent's chess part has just been captured
				tmpBoard[x1][y1] = " ";
				pawnLastRow = true;
				if (color == 0) {
					scoreWhite++;
				}
				else
					scoreBlack++;
			}

		if (tmpBoard[x2][y2].equals("P")) {
			if(color == 0)
				scoreWhite++;
			else
				scoreBlack++;
		}


		// otherwise
		if(!pawnLastRow)
		{
			if(chesspart.equals("P")) {
				if (color == 0) {
					scoreWhite++;
				}
				else
					scoreBlack++;
			}
			else if(chesspart.equals("K")) {
				if (color == 0) {
					scoreWhite+=8;
				}
				else
					scoreBlack+=8;
			}
			else if(chesspart.equals("R")) {
				if (color == 0) {
					scoreWhite+=3;
				}
				else
					scoreBlack+=3;
			}
			tmpBoard[x2][y2] = tmpBoard[x1][y1];
			tmpBoard[x1][y1] = " ";
		}


		//		// check if a prize has been added in the game
		//		if(prizeX != noPrize)
		//			board[prizeX][prizeY] = "P";
		//	}




		//	tmpBoard[a][b] = " ";
		//	if (tmpBoard[c][d].equals("P")) {
		//		if(randomNumberGenerator(1, 100) > 5) {
		//			if (color == 0) 
		//				scoreWhite+=1;
		//			else
		//				scoreBlack+=1;
		//		}
		//	}


		// add new prize
		//				if (randomNumberGenerator(1, 100) < 20) {
		//					while(true) {
		//						int i = randomNumberGenerator(0, 6);
		//						int j = randomNumberGenerator(0, 4);
		//						if (tmpBoard[i][j].equals(" ")) 
		//							break;
		//						System.out.println("try to find  empty space to put prize");
		//					}
		//				}
		// tmpBoard[c][d] = pawn;
	}

	/**
	 * Finds if the game is over
	 * @return true if game is over and false if game is not over
	 */
	public boolean gameOver() {
		int numOfWhiteKings = 0;
		int numOfBlackKings = 0;
		int numOfWhitePawns = 0;
		int numOfBlackPawns = 0;
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < columns; j++) {
				switch (tmpBoard[i][j]) {
				case "WK":
					numOfWhiteKings++;
					break;
				case "BK":
					numOfBlackKings++;
					break;
				case "BP":
				case "BR":
					numOfBlackPawns++;
					break;
				case "WP":
				case "WR":
					numOfWhitePawns++;
					break;
				default:
					break;
				}
			}
		}

		if (numOfBlackKings == 0 || numOfWhiteKings == 0 || (numOfBlackPawns == 0 && numOfWhitePawns == 0)) 
			return true;
		return false;
	}

	
	/**
	 * Heuristic function that evaluates score for each situation in the board
	 * @param board
	 * @param color
	 * @return the value of each node
	 */
	public int evaluateScore(String [][] board, int color) {

		int numOfBlackPawns = 0;
		int numOfWhitePawns = 0;
		int numOfWhiteKnights = 0;
		int numOfBlackKnights = 0;
		int numOfWhiteKings = 0;
		int numOfBlackKings = 0;

		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < columns; j++) {
				switch (tmpBoard[i][j]) {
				case "WP":
					numOfWhitePawns++;
					break;
				case "WR":
					numOfWhiteKnights++;
					break;
				case "WK":
					numOfWhiteKings++;
					break;
				case "BP":
					numOfBlackPawns++;
				case "BR":
					numOfBlackKnights++;
				case "BK":
					numOfBlackKings++;
				default:
					break;
				}
			}
		}
		int valueOfPawnsWhite = numOfWhitePawns + numOfWhiteKnights*3 + numOfWhiteKings*8;
		int valueOfPawnsBlack = numOfBlackPawns + numOfBlackKnights*3 + numOfBlackKings*8;
		if (game.getMyColor() == 0) 
			return(scoreWhite + valueOfPawnsWhite)-(scoreBlack + valueOfPawnsBlack); 
		return (valueOfPawnsBlack) - (valueOfPawnsWhite);

	}



	/**
	 * FUNCTION that creates a random number between the range min,max
	 * @param min
	 * @param max
	 * @return random number
	 */
	public int randomNumberGenerator(int min, int max) {
		Random ran =new Random();
		return ran.nextInt(max-min+1)+min;
	}

	/**
	 * Function that undoes a move in the tmp board
	 * @param oldBoard the old board that wee want to have
	 * @param oldScoreWhite the old score for white player
	 * @param oldScoreBlack the old score for black player
	 */
	public void undoMove(String[][] oldBoard, int oldScoreWhite, int oldScoreBlack) {
		tmpBoard = equalBoards(oldBoard);
		scoreWhite = oldScoreWhite;
		scoreBlack = oldScoreBlack;
	}


	/**
	 * function for setting a newBoard equal to another board given as iput
	 * @param oldBoard
	 * @return the new board
	 */
	public String[][] equalBoards(String[][] oldBoard) {
		String [][] newBoard = new String[rows][columns];
		for (int i = 0; i < rows; i++)
			for(int j = 0; j < columns; j++)
				newBoard[i][j] = oldBoard[i][j];

		return newBoard;

	}


	/**
	 * Function for returning all the available moves for white player
	 * @param board the current board
	 * @return an Arraylist of all the available moves
	 */
	private ArrayList<String> whiteMoves(String [][] board)
	{
		ArrayList<String> availableMoves = new ArrayList<String>();
		String firstLetter = "";
		String secondLetter = "";
		String move = "";

		for(int i=0; i < rows; i++)
		{
			for(int j=0; j < columns; j++)
			{
				firstLetter = Character.toString(board[i][j].charAt(0));

				// if it there is not a white chess part in this position then keep on searching
				if(firstLetter.equals("B") || firstLetter.equals(" ") || firstLetter.equals("P"))
					continue;

				// check the kind of the white chess part
				secondLetter = Character.toString(board[i][j].charAt(1));

				if(secondLetter.equals("P"))	// it is a pawn
				{

					// check if it can move one vertical position ahead
					if (i != 0) {
						firstLetter = Character.toString(board[i-1][j].charAt(0));
					}

					if(firstLetter.equals(" ") || firstLetter.equals("P"))
					{
						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i-1) + Integer.toString(j);

						availableMoves.add(move);
					}

					// check if it can move crosswise to the left
					if(j!=0 && i!=0)
					{
						firstLetter = Character.toString(board[i-1][j-1].charAt(0));						
						if(!(firstLetter.equals("W") || firstLetter.equals(" ") || firstLetter.equals("P"))) {
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i-1) + Integer.toString(j-1);

							availableMoves.add(move);
						}											
					}

					// check if it can move crosswise to the right
					if(j!=columns-1 && i!=0)
					{
						firstLetter = Character.toString(board[i-1][j+1].charAt(0));
						if(!(firstLetter.equals("W") || firstLetter.equals(" ") || firstLetter.equals("P"))) {

							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i-1) + Integer.toString(j+1);							
							availableMoves.add(move);
						}
					}
				}
				else if(secondLetter.equals("R"))	// it is a rook
				{
					// check if it can move upwards
					for(int k=0; k<rookBlocks; k++)
					{
						if((i-(k+1)) < 0)
							break;

						firstLetter = Character.toString(board[i-(k+1)][j].charAt(0));

						if(firstLetter.equals("W"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i-(k+1)) + Integer.toString(j);

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("B") || firstLetter.equals("P"))
							break;
					}

					// check if it can move downwards
					for(int k=0; k<rookBlocks; k++)
					{
						if((i+(k+1)) == rows)
							break;

						firstLetter = Character.toString(board[i+(k+1)][j].charAt(0));

						if(firstLetter.equals("W"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i+(k+1)) + Integer.toString(j);

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("B") || firstLetter.equals("P"))
							break;
					}

					// check if it can move on the left
					for(int k=0; k<rookBlocks; k++)
					{
						if((j-(k+1)) < 0)
							break;

						firstLetter = Character.toString(board[i][j-(k+1)].charAt(0));

						if(firstLetter.equals("W"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i) + Integer.toString(j-(k+1));

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("B") || firstLetter.equals("P"))
							break;
					}

					// check of it can move on the right
					for(int k=0; k<rookBlocks; k++)
					{
						if((j+(k+1)) == columns)
							break;

						firstLetter = Character.toString(board[i][j+(k+1)].charAt(0));

						if(firstLetter.equals("W"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i) + Integer.toString(j+(k+1));

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("B") || firstLetter.equals("P"))
							break;
					}
				}
				else // it is the king
				{
					// check if it can move upwards
					if((i-1) >= 0)
					{
						firstLetter = Character.toString(board[i-1][j].charAt(0));

						if(!firstLetter.equals("W"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i-1) + Integer.toString(j);

							availableMoves.add(move);	
						}
					}

					// check if it can move downwards
					if((i+1) < rows)
					{
						firstLetter = Character.toString(board[i+1][j].charAt(0));

						if(!firstLetter.equals("W"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i+1) + Integer.toString(j);

							availableMoves.add(move);	
						}
					}

					// check if it can move on the left
					if((j-1) >= 0)
					{
						firstLetter = Character.toString(board[i][j-1].charAt(0));

						if(!firstLetter.equals("W"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i) + Integer.toString(j-1);

							availableMoves.add(move);	
						}
					}

					// check if it can move on the right
					if((j+1) < columns)
					{
						firstLetter = Character.toString(board[i][j+1].charAt(0));

						if(!firstLetter.equals("W"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i) + Integer.toString(j+1);

							availableMoves.add(move);	
						}
					}
				}			
			}	
		}
		return availableMoves;
	}


	/**
	 * function that gives as all the available moves of a black player
	 * @param board the current board
	 * @return all the available moves in an arraylist
	 */
	private ArrayList<String> blackMoves(String [][] board)
	{
		ArrayList<String> availableMoves = new ArrayList<String>();
		String firstLetter = "";
		String secondLetter = "";
		String move = "";

		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				firstLetter = Character.toString(board[i][j].charAt(0));

				// if it there is not a black chess part in this position then keep on searching
				if(firstLetter.equals("W") || firstLetter.equals(" ") || firstLetter.equals("P"))
					continue;

				// check the kind of the white chess part
				secondLetter = Character.toString(board[i][j].charAt(1));

				if(secondLetter.equals("P"))	// it is a pawn
				{

					// check if it can move one vertical position ahead
					if (i !=6) {
						firstLetter = Character.toString(board[i+1][j].charAt(0));
					}

					if(firstLetter.equals(" ") || firstLetter.equals("P"))
					{
						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i+1) + Integer.toString(j);

						availableMoves.add(move);
					}

					// check if it can move crosswise to the left
					if(j!=0 && i!=rows-1)
					{
						firstLetter = Character.toString(board[i+1][j-1].charAt(0));

						if(!(firstLetter.equals("B") || firstLetter.equals(" ") || firstLetter.equals("P"))) {
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i+1) + Integer.toString(j-1);

							availableMoves.add(move);
						}																	
					}

					// check if it can move crosswise to the right
					if(j!=columns-1 && i!=rows-1)
					{
						firstLetter = Character.toString(board[i+1][j+1].charAt(0));

						if(!(firstLetter.equals("B") || firstLetter.equals(" ") || firstLetter.equals("P"))) {
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i+1) + Integer.toString(j+1);

							availableMoves.add(move);
						}



					}
				}
				else if(secondLetter.equals("R"))	// it is a rook
				{
					// check if it can move upwards
					for(int k=0; k<rookBlocks; k++)
					{
						if((i-(k+1)) < 0)
							break;

						firstLetter = Character.toString(board[i-(k+1)][j].charAt(0));

						if(firstLetter.equals("B"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i-(k+1)) + Integer.toString(j);

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("W") || firstLetter.equals("P"))
							break;
					}

					// check if it can move downwards
					for(int k=0; k<rookBlocks; k++)
					{
						if((i+(k+1)) == rows)
							break;

						firstLetter = Character.toString(board[i+(k+1)][j].charAt(0));

						if(firstLetter.equals("B"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i+(k+1)) + Integer.toString(j);

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("W") || firstLetter.equals("P"))
							break;
					}

					// check if it can move on the left
					for(int k=0; k<rookBlocks; k++)
					{
						if((j-(k+1)) < 0)
							break;

						firstLetter = Character.toString(board[i][j-(k+1)].charAt(0));

						if(firstLetter.equals("B"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i) + Integer.toString(j-(k+1));

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("W") || firstLetter.equals("P"))
							break;
					}

					// check of it can move on the right
					for(int k=0; k<rookBlocks; k++)
					{
						if((j+(k+1)) == columns)
							break;

						firstLetter = Character.toString(board[i][j+(k+1)].charAt(0));

						if(firstLetter.equals("B"))
							break;

						move = Integer.toString(i) + Integer.toString(j) + 
								Integer.toString(i) + Integer.toString(j+(k+1));

						availableMoves.add(move);

						// prevent detouring a chesspart to attack the other
						if(firstLetter.equals("W") || firstLetter.equals("P"))
							break;
					}
				}
				else // it is the king
				{
					// check if it can move upwards
					if((i-1) >= 0)
					{
						firstLetter = Character.toString(board[i-1][j].charAt(0));

						if(!firstLetter.equals("B"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i-1) + Integer.toString(j);

							availableMoves.add(move);	
						}
					}

					// check if it can move downwards
					if((i+1) < rows)
					{
						firstLetter = Character.toString(board[i+1][j].charAt(0));

						if(!firstLetter.equals("B"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i+1) + Integer.toString(j);

							availableMoves.add(move);	
						}
					}

					// check if it can move on the left
					if((j-1) >= 0)
					{
						firstLetter = Character.toString(board[i][j-1].charAt(0));

						if(!firstLetter.equals("B"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i) + Integer.toString(j-1);

							availableMoves.add(move);	
						}
					}

					// check if it can move on the right
					if((j+1) < columns)
					{
						firstLetter = Character.toString(board[i][j+1].charAt(0));

						if(!firstLetter.equals("B"))
						{
							move = Integer.toString(i) + Integer.toString(j) + 
									Integer.toString(i) + Integer.toString(j+1);

							availableMoves.add(move);	
						}
					}
				}			
			}	
		}
		return availableMoves;
	}


}
