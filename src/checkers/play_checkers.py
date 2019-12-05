import numpy as np
class CheckersAI():
    '''Keeps track of game progress and uses Minimax with alpha beta pruning to find best move for the Baxter robot'''
    def __init__(self):
        self.game_over = False
        self.baxter_color = 'black'#None

        if self.baxter_color == 'black':
            self.not_baxter = 'red'
        elif self.baxter_color == 'red':
            self.not_baxter = 'black'
        self.whos_turn = 'black' #black goes first
        self.noCapture = 0
        self.winner = None
        self.alpha = float('-inf')
        self.beta = float('+inf')
        state = [[-1, 0,-1, 0,-1, 0,-1, 0],
                 [ 0,-1, 0,-1, 0,-1 ,0,-1],
                 [-1, 0,-1, 0,-1, 0,-1, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 1, 0, 1, 0, 1, 0, 1],
                 [ 1, 0, 1, 0, 1, 0, 1, 0],
                 [ 0, 1, 0, 1, 0, 1, 0, 1]]
        self.board = Board()

    def give_command(self,state,color):
        '''ARGS:
                board: Board object that stores the positions of the pieces on the board, used to check if the move is legal
                color: designates what color the player is, used for get_moves
           Returns:
                start: Grid position of the piece to move
                goal: Grid position of where to move the piece to
                captured: returns position of any captured pieces'''
        valid = False
        captured = []
        state = self.board.world_to_grid(state)
        self.board.get_piece_count(state)
        moves, cap = self.board.get_moves(state,color)
        print('Legal Moves in python indicies so add 1 to each value: '+str(moves))
        #print(moves[0][0])
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
                            captured.append(c)
            if not valid:
                print('\nEntered an illegal move, please enter a different move.\n')
        return start, goal, captured

    def switch_turn(self):
        '''Sets variable for which players turn it is to the other player'''
        if self.whos_turn == 'Red':
            self.whos_turn = 'Black'
        else:
            self.whos_turn = 'Red'

    def is_game_over(self,state):
        self.board.get_piece_count(state)
        #print(self.board.black_piece_count)
        #print(self.board.red_piece_count)
        if self.board.red_piece_count == 0:
            if self.baxter_color == 'black':
                self.winner = 'baxter'
            else:
                self.winner = 'not_baxter'
            self.game_over = True
        elif self.board.black_piece_count == 0:
            if self.baxter_color == 'red':
                self.winner = 'baxter'
            else:
                self.winner = 'not_baxter'
            self.game_over = True
        elif self.board.get_moves(state,self.whos_turn) == None:
            self.switch_turn()
            if self.whos_turn == self.baxter_color:
                self.winner = 'baxter'
            else:
                self.winner = 'not_baxter'
            self.game_over = True
        elif self.noCapture > 50:
            self.winner = 'Draw'
            self.game_over = True
        #if self.game_over:
        #    if self.winner == 'Draw':
        #        print()
        #    print('The winner is '+str(self.winner))
        #return self.game_over

    def grid_to_world(self,move,captured):
        '''Converts moves from grid coordinates to index of 1-d list refering to board positions
        assumes left to right, bottom to top from baxter's perspective
        ARGS:
            move: list of start and goal postion ([[0,0],[2,2]])
            captured: list of start and captured piece locations ([[0,0],[1,1]])
        Returns:
            movelist: list of indeices corresponding to grid coordinates'''
        movelist = [move[0][0]*8+move[0][1],move[1][0]*8+move[1][1]]
        for cap in captured:
            movelist.append(cap[1][0]*8+cap[1][1])
        print('Move list: '+str(movelist))
        return movelist




    def make_move(self,move,cap,state):
        '''make a array of board states (nodes) after all legal moves starting from current state. include whether or not the node is terminal'''
        print(state)
        if len(state) > 8:
            print('world_to_grid')
            state = self.board.world_to_grid(state)
        print('moves: '+str(move))
        #if len(cap) != 0:
        print('captures: '+str(cap))
            #cap = cap[0]
        #for move in moves:
        print(move[0])
        w = 0
        print(state)#np.array(self.board.state)) #initial state
        #state = state[:]#self.board.state[:]
        state[move[0][0]][move[0][1]] = 0
        state[move[1][0]][move[1][1]] = self.board.p #should still be the last players value

        if len(cap) != 0 and cap[0][0] == move[0]:
            for piece in cap:
                print('Capture '+str(piece))
                state[piece[1][0]][piece[1][1]] = 0
                self.noCapture = 0
        else:
            self.noCapture += 1 #if it counts too high its a draw
        self.is_game_over(state)
        if self.game_over:
            print('Game over: Winner: '+str(self.winner))
            w = 1
        #nodes.append([move,temp_state,w])
        #self.board.state = self.board.init_state[:]
        #nodes = temp_state[:]
        #print(np.array(nodes))
        return state


    def minimax(self,state_list):
        '''Minimax algorithm to get best moves for baxter
            ARGS: max: (bool) is maximizing player?'''
        state = np.array(self.board.world_to_grid(state_list))
        moves,cap,s = self.board.get_moves(state,self.baxter_color)
        if len(moves) == 1: return moves, cap
        else:
            print(state)
            self.path = []
            bestvalue = self.prune(self.alpha,self.beta,state,True,3)
            print(bestvalue)
        best_move = []





    def prune(self,alpha,beta,node,max,level): #based on pseudocode from https://www.hackerearth.com/blog/developers/minimax-algorithm-alpha-beta-pruning/
        '''Alpha beta pruning to optimize minimax'''
        #self.board.world_to_grid(node)
        #self.board.init_state = node
        val = 0
        self.is_game_over(node)
        print('black: '+str(self.board.black_piece_count))
        print('red: '+str(self.board.red_piece_count))
        if self.game_over or level == 0: #return utility of the node if terminal node
            if self.winner == 'baxter': #returns count of baxter's pieces
                if self.baxter_color == 'black':
                    val = self.board.black_piece_count
                else:
                    val = self.board.red_piece_count
            elif self.winner == 'not_baxter': #return the count of not baxter's pieces
                if self.not_baxter == 'black':
                    val = self.board.black_piece_count
                else:
                    val = self.board.red_piece_count
            else:
                val = 0 #if draw score is zero
            self.path.append([node,val])
            return val

        if max: #baxter is maximizing
            print('Max turn')
            print(self.baxter_color)
            moves,cap,s = self.board.get_moves(node,self.baxter_color) #gets moves for current node
            print('Nodes:')
            print(moves)
            #print(np.array(nodes[0][1]))
            print(np.array(node))
            print('\n')

            val = alpha
            for move in moves:
                child = self.make_move(move,cap,node)
                print('child:\n '+str(np.array(child)))
                val = max(self.prune(alpha,beta,child,False,level-1),val)
                print('value: '+str(val))
                print('beta: '+str(beta))
                if beta <= val: #don't update alpha and prune if its greater than beta
                    print('returning val')
                    return val
                alpha = max(alpha,val)
            return val
        else: #minimizing player
            print(self.not_baxter)
            print('before')
            print(node)
            moves, cap, test_s = self.board.get_moves(node,self.not_baxter) #gets moves for current node
            print('after')
            print(node)
            print(test_s)
            val = beta
            for move in moves:
                child = self.make_move(move,cap,node)
                print('child:\n '+str(np.array(child)))
                val = min(self.prune(alpha,beta,test_s,False,level-1),val)
                print('value: '+str(val))
                print('beta: '+str(beta))
                if val <= alpha: #don't update alpha and prune if its greater than beta
                    print('returning val')
                    return val
                beta = min(beta,val)
            return val


class Board():
    '''Keeps track of board state and move generation. Initialized and called only in the CheckersAI class'''
    def __init__(self):#,state):
        self.red_piece_count = 0
        self.black_piece_count = 0
        #self.init_state = state
        #self.state = state

    def get_piece_count(self,state):
        self.red_piece_count = 0
        self.black_piece_count = 0
        for row in state:
            for col_ele in row:
                if col_ele < 0:
                    self.black_piece_count += 1
                elif col_ele > 0:
                    self.red_piece_count += 1

    def world_to_grid(self,list):
        '''recieve list[0,63], index refers to grid square (inorder),elements are 'empty','color1',color2,'color1_king',color2_king'''
        '''Converts from computer vision input to grid array. Will be given an (x,y) position and color for that position.'''
        state = [[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]]
        r = 0
        c = 0
        for ele in list:
            #print('element: '+str(ele))
            state[r][c] = ele
            #print(state)
            if c == 7:
                r += 1
                c = 0
            else:
                c += 1
        self.init_state = state
        self.get_piece_count(state)
        return state



    def get_moves(self,state,player):
        '''ARGS: player: current players color, used to check moves for only that players moves
        Returns list of all legal moves and captured pieces as tuple pairs (Ex. [[start],[goal]] and [[start],[captured_position]])'''
        def can_jump(row,col,step_r,step_c,state):
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
                if state[row+(2*step_r)][col+(2*step_c)] == 0 and \
                state[row+step_r][col+step_c] == self.not_p or state[row+step_r][col+step_c] == self.not_p*2:  #check if space after jump is empty and if it is the opponents piece
                    #capture condition met, save position of jump and captured piece
                    move = [[row,col],[row+(2*step_r),col+(2*step_c)]]
                    captured = [[row,col],[row+step_r,col+step_c]]
                    cap = True
            return move,captured,cap

        def can_step(r,c,state):
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
            #set possible steps based on whether a pawn or king and which player
            if state[r][c] == self.p and self.p == -1:
                step = [[1,1],[1,-1]]
            elif state[r][c] == self.p and self.p == 1:
                step = [[-1,1],[-1,-1]]
            if state[r][c] == self.p*2:
                step = [[1,1],[1,-1],[-1,1],[-1,-1]]
                #print('King')
            for s in step: #take a step in valid directions
                stepc = s[1]
                stepr = s[0]
                if c+stepc >= 0 and c+stepc <= 7 and r+stepr >= 0 and r+stepr <= 7: # Checks that the step won't take piece out of bounds
                    '''Single Step'''
                    if state[r+stepr][c+stepc] == 0: #if the diagonal square is empty, add step to list of legal moves
                        moves.append([[r,c],[r+stepr,c+stepc]])
                    else: #If not empty, try to make a jump
                        move, cap, cap_success = can_jump(r,c,stepr,stepc,state) #single jump
                        moves = [move]
                        if len(cap) != 0:
                            capture = [cap] #store captured piece position if successful jump
                        '''Double jump'''
                        if cap_success: #check for double jump
                            for s2 in step: #check all possible step directions
                                stepc2 = s2[1]
                                stepr2 = s2[0]
                                parent = move[0] #save start position from first jump
                                temp, dcap,cap_success = can_jump(move[1][0],move[1][1],stepr2,stepc2,state) #double jump
                                if cap_success:
                                    moves[0][1] = temp[1] #replace previously saved end position with end position after double jump
                                    capture.append([parent,dcap[1]])

            return moves, capture

        '''def flip_board(state):
            Args:
                 state: a array representation of a board state.
               Returns:
                 flip_state: the same board with the first index corresponding to the corner diagonally opposite.
                Flips the board so the board is in the perspective of whichever players turn it is
            flip_state = state[::-1]
            for l in range(len(flip_state)):
                flip_state[l] = flip_state[l][::-1]
            return flip_state'''

        #currently assuming list
        self.get_piece_count(state)
        moves = []
        temp = []
        capture = []
        steps = [[1,-1],[1,1]]
        self.step_r = 1
        if player.lower() == 'black': #state is always read in starting with dark color first since black player goes first
            #Black is -1
            self.p = -1
            self.not_p = 1
        else:
            #state = flip_board(state)
            self.p = 1 #Red is +1
            self.not_p = -1
        #cycle through grid cells
        for r in range(8):
            for c in range(8):
                mK,capK = can_step(r,c,state)
                temp_moves = []
                #print(mK)
                for i in range(len(mK)):
                    if len(mK[i]) != 0:
                        temp_moves.append(mK[i])
                        moves.append(mK[i])
                for i in range(len(capK)):
                    if len(capK[i]) != 0:
                        capture.append(capK[i])
                        capture_move = temp_moves
                        #return temp_moves, capture
                        #for move in mk
        if len(capture) != 0:
            return capture_move, capture

        return moves,capture
