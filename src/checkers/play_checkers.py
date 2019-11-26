
class CheckersAI():
    '''Keeps track of game progress and uses Minimax with alpha beta pruning to find best move for the Baxter robot'''
    def __init__(self):
        self.game_over = False
        self.whos_turn = 'Red'
        self.noCaptureCounter = 0
        self.winner = None
        self.alpha = float('-inf')
        self.beta = float('+inf')

    def give_command(self,board,color):
        '''ARGS:
                board: Board object that stores the positions of the pieces on the board, used to check if the move is legal
                color: designates what color the player is, used for get_moves
           Returns:
                start: Grid position of the piece to move
                goal: Grid position of where to move the piece to
                captured: returns position of any captured pieces'''
        valid = False
        captured = []
        moves, cap = board.get_moves(color)
        #print('Legal Moves: '+str(moves))
        #print(cap)
        while not valid:
            start_in = raw_input('Enter position of piece you want to move or q to quit: ')
            if start_in == 'q':
                print('Player quit')
                return None,None,None
            goal_in = raw_input('Enter where you want to move the piece to: ')
            start = start_in.split(',')
            goal = goal_in.split(',')
            for i in range(len(start)):
                try:
                    start[i] = int(start[i])-1 #convert to python counting convention
                    goal[i] = int(goal[i])-1 #convert to python counting convention
                except ValueError:
                    print('\nEntered move is not in valid format. Please enter two integers seperated by a comma\n')
                    valid = False
            for move in moves:
                #check if legal move
                if [start,goal] == move:
                    print('Moving...')
                    valid = True
                    for c in cap:
                        if start == c[0]:
                            captured.append(c[1])
            if not valid:
                print('\nEntered an illegal move, please enter a different move.\n')


        return start, goal, captured

    def switch_turn(self):
        '''Sets variable for which players turn it is to the other player'''
        if self.whos_turn == 'Red':
            self.whos_turn = 'Black'
        else:
            self.whos_turn = 'Red'

    def is_game_over(self):
        if self.red_piece_count == 0:
            self.winner = 'Black'
            self.game_over = True

        if self.black_piece_count == 0:
            self.winner = 'Red'
            self.game_over = True

    def take_turn(self):
        '''Get board state
           check if game is over
           Get legal moves for that state
           use minimax
                get sequnces of legal moves and their score
                prune
                get bestvalue
            return move
            check if game is over
            '''
        pass

    def minimax(self):
        pass

    def prune(self):
        pass

class Board():
    def __init__(self,state):
        self.red_piece_count = 12
        self.black_piece_count = 12
        self.state = state


    def world_to_grid(self):
        '''Converts from computer vision input to grid array. Will be given an (x,y) position and color for that position.'''
        pass

    def get_moves(self,player):

        def can_jump(row,col,step_r,step_c):
            '''ARGS:
                row: current row
                col: current column
                step_r: directions to step row-wise
                step_c: directions to step column-wise
               Returns:
                move: (start, goal) pair (if any) of legal moves
                captured: (start,captured piece position) pair of any captured pieces
                cap: (bool) whether any pieces were captured or not
            Checks if step in given direction keeps piece in bound, if it does, checks if there is a place to jump, and the jumped piece is the other player's
                '''
            captured = []
            move = []
            cap = False
            if col+(2*step_c) >= 0 and col+(2*step_c) <= 7 and row+(2*step_r) >= 0 and row+(2*step_r) <= 7:
                if self.state[row+(2*step_r)][col+(2*step_c)] == 0 and \
                self.state[row+step_r][col+step_c] == self.not_p or self.state[row+step_r][col+step_c] == self.not_p*2:  #check if space after jump is empty and if it is the opponents piece
                    #capture condition met, save position of jump and captured piece
                    move = [[row,col],[row+(2*step_r),col+(2*step_c)]]
                    captured = [[row,col],[row+step_r,col+step_c]]
                    cap = True
                    if self.p == -1: #reduce pices count
                        self.black_piece_count -= 1
                    else:
                        self.red_piece_count -= 1
            return move,captured,cap

        def can_step(r,c):
            ''' Args:
                 r: current row index
                 c: current column index
                 step: either 1,-1, or both, states which direction side to side the piece can move diagonally
                Returns:
                 moves: list of (start, goal) pairs (if any) of legal moves
                 capture: list of (start,captured piece position) pairs of any captured pieces
                Checks if a step is possible is each possible direction for a type of piece. if step is not possible, checks for jump.
                if jump was successful, checks for possible double jumps.'''
            moves = []
            capture = []
            step = []
            #set possible steps based on whether a pawn or king
            if self.state[r][c] == self.p:
                step = [[1,1],[1,-1]]
            if self.state[r][c] == self.p*2:
                step = [[1,1],[1,-1],[-1,1],[-1,-1]]
                #print('King')
            for s in step: #take a step in valid directions
                stepc = s[1]
                stepr = s[0]
                if c+stepc >= 0 and c+stepc <= 7 and r+stepr >= 0 and r+stepr <= 7: # Checks that the step won't take piece out of bounds
                    '''Single Step'''
                    if self.state[r+stepr][c+stepc] == 0: #if the diagonal square is empty, add step to list of legal moves
                        moves.append([[r,c],[r+stepr,c+stepc]])
                    else: #If not empty, try to make a jump
                        move, cap, cap_success = can_jump(r,c,stepr,stepc) #single jump
                        moves = [move]
                        if len(cap) != 0:
                            capture = [cap] #store captured piece position if successful jump
                        '''Double jump'''
                        if cap_success: #check for double jump
                            for s2 in step: #check all possible step directions
                                stepc2 = s2[1]
                                stepr2 = s2[0]
                                parent = move[0] #save start position from first jump
                                temp, dcap,cap_success = can_jump(move[1][0],move[1][1],stepr2,stepc2) #double jump
                                if cap_success:
                                    moves[0][1] = temp[1] #replace previously saved end position with end position after double jump
                                    capture.append([parent,dcap[1]])

            return moves, capture

        def flip_board(state):
            '''Args:
                 state: a array representation of a board state.
               Returns:
                 flip_state: the same board with the first index corresponding to the corner diagonally opposite.
                Flips the board so the board is in the perspective of whichever players turn it is'''
            flip_state = state[::-1]
            for l in range(len(flip_state)):
                flip_state[l] = flip_state[l][::-1]
            return flip_state

        #currently assuming list
        moves = []
        temp = []
        capture = []
        steps = [[1,-1],[1,1]]
        self.step_r = 1
        if player == 'Black': #state is always read in starting with dark color first since black player goes first
            #Black is -1
            self.p = -1
            self.not_p = 1
        else:
            self.state = flip_board(self.state)
            self.p = 1 #Red is +1
            self.not_p = -1
        #cycle through grid cells
        for r in range(8):
            for c in range(8):
                mK,capK = can_step(r,c)
                for i in range(len(mK)):
                    if len(mK[i]) != 0:
                        moves.append(mK[i])
                for i in range(len(capK)):
                    if len(capK[i]) != 0:
                        capture.append(capK[i])

        return moves, capture
