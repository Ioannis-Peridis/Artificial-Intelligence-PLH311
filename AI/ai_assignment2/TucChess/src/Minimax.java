
import java.util.ArrayList;
import java.util.Iterator;
//import java.util.Iterator;
import java.util.Random;

/*
 * Constructor of minimax
 */
public class Minimax {
	private int rookBlocks = 3;		// rook can move towards <rookBlocks> blocks in any vertical or horizontal direction
	//	private int nTurns = 0;
	//	private int nBranches = 0;
	//	private int noPrize = 9;
	private int rows = 7;
	private int columns = 5;
	private World game;
	private String[][] tmpBoard ;
	public static int MAX = 1000000;
	public int scoreWhite = 0;
	public int scoreBlack = 0;
	boolean ABPrunning = false;
	public Minimax(World game) {
		tmpBoard = new String[rows][columns];
		for(int i=0; i < rows; i++)
			for(int j=0; j < columns; j++)
				tmpBoard[i][j]= " ";
		this.game = game;
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
	 * Function that generates a random number between a given range 
	 * @param min bottom of range
	 * @param max top of range
	 * @return the random number that we generated
	 */
	public int randomNumberGenerator(int min, int max) {
		Random ran =new Random();
		return ran.nextInt(max-min+1)+min;
	}


	/**
	 * undo a move previously done 
	 * @param oldBoard the old board
	 */
	public void undoMove(String[][] oldBoard) {
		tmpBoard = equalBoards(oldBoard);
	}

	/**
	 * Function chooses which algorithm is going to be executed
	 * * If ABPrunning is true starts minimax with AB Prunning
	 * * Else simple minimax
	 * @return the move that we are going to execute
	 */
	public String selectMiniMax() {
		scoreWhite = game.scoreWhite;
		scoreBlack = game.scoreBlack;
		Node node;
		tmpBoard =equalBoards(game.getBoard());
		if (ABPrunning) {
			node = alphaBetaPrunning(7, game.getMyColor(), -MAX, +MAX);
		}
		else
			node = miniMaxAlgo(5,game.getMyColor());
		//System.out.println(node.move);
		return node.move; 
	}

	/**
	 * Function that executes minimax algorithm. Takes as input an integer depth and an integer that specifies the color of the player
	 * @param depth is how deep in the tree of the minimax we are going to search for the best move
	 * @param color is the color kof the player. 0 for white and 1 for black
	 * @return the node that represents the best possible move for the specific depth. This depends on our heuristic function
	 */
	public Node miniMaxAlgo(int depth, int color) {
		ArrayList<String> availableMoves = new ArrayList<String>();
		String[][] oldBoard = equalBoards(tmpBoard);
		int minMaxValue = 0;
		String bestMove = " ";
		if ( gameOver() || depth == 0) 
			return new  Node(null, evaluateScore(tmpBoard, color));

		if (color == 0) {
			availableMoves = getMoves(tmpBoard,color);
			minMaxValue = -MAX;
			for (String move : availableMoves) {
				oldBoard = equalBoards(tmpBoard);
				makeMove(move,color);
				Node node = miniMaxAlgo(depth-1, changeColor(color));
				tmpBoard = equalBoards(oldBoard);
				if (node.value > minMaxValue) {
					minMaxValue = node.value;
					bestMove = move;
				}
			}
		}
		else {
			availableMoves = getMoves(tmpBoard, color);
			minMaxValue = MAX;
			for (String move : availableMoves) {
				oldBoard = equalBoards(tmpBoard);
				makeMove(move,color);
				// color has already changed
				Node node = miniMaxAlgo(depth-1, changeColor(color));
				tmpBoard = equalBoards(oldBoard);
				if (node.value < minMaxValue) {
					minMaxValue = node.value;
					bestMove = move;
				}
			}
		}

		return new Node(bestMove, minMaxValue);
	}

	/**
	 * Function for abprunning
	 * @param depth of the tree
	 * @param color of the player
	 * @param a the a of ABprunning
	 * @param b the b of ABprunning
	 * @return node that represents best possible move 
	 */
	public Node alphaBetaPrunning(int depth, int color, int a, int b) {
		double start = System.currentTimeMillis();
		ArrayList<String> availableMoves = new ArrayList<String>();
		String[][] oldBoard = equalBoards(tmpBoard);
		int minMaxValue = 0;
		String bestMove = " ";
		if ( gameOver() || depth == 0) 
			return new  Node(null, evaluateScore(tmpBoard, color));

		if (color == 0) {
			availableMoves = getMoves(tmpBoard,color);
			minMaxValue = -MAX;
			for (String move : availableMoves) {
				oldBoard = equalBoards(tmpBoard);
				makeMove(move,color);
				Node node = alphaBetaPrunning(depth-1, changeColor(color), a, b);
				tmpBoard = equalBoards(oldBoard);
				if (node.value > minMaxValue) {
					minMaxValue = node.value;
					bestMove = move;
				}
				if (a < node.value) 
					a = node.value;
				if(b <= a)
					break;
			}
		}
		else {
			availableMoves = getMoves(tmpBoard, color);
			minMaxValue = MAX;
			for (String move : availableMoves) {
				oldBoard = equalBoards(tmpBoard);
				makeMove(move,color);
				// color has alrady changed
				Node node = alphaBetaPrunning(depth-1, changeColor(color), a , b);
				tmpBoard = equalBoards(oldBoard);
				if (node.value < minMaxValue) {
					minMaxValue = node.value;
					bestMove = move;
				}
				if(b > node.value)
					b = node.value;
				if(b <= a)
					break;
			}
		}
		return new Node(bestMove, minMaxValue);
	}
	
	
	/**
	 * Function that changes the color. If is black make it white and if is white make it black
	 * @param color current's player color
	 * @return color of the player playing next
	 */
	public int changeColor(int color) {
		if (color == 0) 
			return 1;
		return 0;
	}


	/**
	 * Function that get all the available moves depending on the color of the player
	 * @param board current board that we are searching for the available moves
	 * @param color of the player
	 * @return all possible moves of the player with the color given as input
	 */
	public ArrayList<String> getMoves(String [][] board, int color){
		if (color == 0) 
			return this.whiteMoves(tmpBoard);
		return this.blackMoves(tmpBoard);

	}


	/**
	 * Function that returns a new board equal to the input
	 * @param oldBoard the old board
	 * @return the old board
	 */
	public String[][] equalBoards(String[][] oldBoard) {
		String [][] newBoard = new String[rows][columns];
		for (int i = 0; i < rows; i++)
			for(int j = 0; j < columns; j++)
				newBoard[i][j] = oldBoard[i][j];

		return newBoard;

	}

	/**
	 * Finds if the game is over
	 * @return true if game is over and false if game is not over
	 */
	public boolean gameOver() {
		int numOfWhiteKings = 0;
		int numOfBlackKings = 0;
		int numOfBlackPawns = 0;
		int numOfWhitePawns = 0;
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < columns; j++) {
				switch (tmpBoard[i][j]) {
				case "WK":
					numOfWhiteKings++;
					break;
				case "BK":
					numOfBlackKings++;
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
		if (color == 0) 
			return (scoreWhite + valueOfPawnsWhite) - ( scoreBlack+ valueOfPawnsBlack); 
		return ( scoreBlack+ valueOfPawnsBlack) - (scoreWhite + valueOfPawnsWhite);

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
