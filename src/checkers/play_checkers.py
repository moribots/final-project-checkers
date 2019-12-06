import numpy as np
import copy
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

    def give_command(self,state_list):#,color(used for testing)):
        '''ARGS:
                state_list:string that stores the positions of the pieces on the board, output from CV node
           Returns:
                movelist: returns list of indicies for the pieces to move [start, goal, captured1,captured2,...]
        Takes state_list (string) converts it to integer board array. Deteremies baxter's color. Uses state array and
        baxter's color to get the legal moves. Waits for user to input a start and end goal then checks if this is a legal move.
        if it is a legal move, takes that move and any captured piece positions and converts to world posiiton index, which it returns.'''
        valid = False
        captured = []
        state = self.board.world_to_grid(state_list)
        self.board.get_piece_count(state)
        color = self.board.baxter_color
        moves, cap,p = self.board.get_moves(state,color)#self.board.baxter_color)
        print(np.array(state))
        print('Legal Moves in python indicies so add 1 to each value: '+str(moves))
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
                except ValueError, IndexError:
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
        print('Start: [%i,%i], Goal: [%i,%i]'%(start[0]+1,start[1]+1,goal[0]+1,goal[1]+1))
        print('Captured: '+str(captured))
        after_move = self.make_move([start,goal],captured,state,p)
        print(np.array(after_move[0]))
        return self.grid_to_world([start, goal], captured)

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
            if self.board.baxter_color == 'black':
                self.winner = 'baxter'
            else:
                self.winner = 'not_baxter'
            self.game_over = True
        elif self.board.black_piece_count == 0:
            if self.board.baxter_color == 'red':
                self.winner = 'baxter'
            else:
                self.winner = 'not_baxter'
            self.game_over = True
        elif self.board.get_moves(state,self.whos_turn) == None:
            self.switch_turn()
            if self.whos_turn == self.board.baxter_color:
                self.winner = 'baxter'
            else:
                self.winner = 'not_baxter'
            self.game_over = True
        elif self.noCapture > 50:
            self.winner = 'Draw'
            self.game_over = True
        # print('Is game over? '+str(self.game_over))

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

    def make_move(self,move,cap,state,p):
        '''make a array of board states (nodes) after all legal moves starting from current state. include whether or not the node is terminal'''
        if len(state) > 8:
            #print('world_to_grid')
            state = self.board.world_to_grid(state)
        #print('moves: '+str(move))
        #print('captures: '+str(cap))
        K = 1
        if p == 1 and move[1][0] == 0 or p == -1 and move[1][0] == 7 or state[move[0][0]][move[0][1]] == p*2:
            K = 2
        state[move[0][0]][move[0][1]] = 0
        state[move[1][0]][move[1][1]] = K*p #should still be the last players value
        captured = []
        if len(cap) != 0 and cap[0][0] == move[0]:
            for piece in cap:
                # print('Capture '+str(piece))
                captured.append(state[piece[1][0]][piece[1][1]])
                state[piece[1][0]][piece[1][1]] = 0
                self.noCapture = 0
        else:
            self.noCapture += 1 #if it counts too high its a draw
        self.is_game_over(state)
        #if self.game_over:
            # print('Game over: Winner: '+str(self.winner))
        #nodes.append([move,temp_state,w])
        return state,captured

    def undo_move(self,move,cap,cap_piece,state,player):
        K = 1
        #player = self.board.p #Assumes winning player made the last move
        #if self.game_over == False: #max level reached so other play made the last move
        #    player = self.board.not_p
        if state[move[1][0]][move[1][1]] == player*2: #only have to worry about piece before move
            K = 2
        state[move[1][0]][move[1][1]] = 0
        state[move[0][0]][move[0][1]] = K*player #should still be the last players value
        if len(cap) != 0 and cap[0][0] == move[0]:
            for i in range(len(cap)):
                #print('Capture '+str(piece))
                state[cap[i][1][0]][cap[i][1][1]] = cap_piece[i]
                self.noCapture -= 1
        # print(np.array(state))
        return state

    def minimax(self,state_list):
        '''Minimax algorithm to get best moves for baxter
            ARGS: max: (bool) is maximizing player?'''
        state = self.board.world_to_grid(state_list)
        best_move = []
        moves,cap,p = self.board.get_moves(state,self.board.baxter_color)
        if len(moves) == 1: return self.grid_to_world(moves,cap) #captured moves are forced
        else:
            self.path = []
            bestvalue = self.prune(self.alpha,self.beta,state,'max',0)
            print('Best_value: '+str(bestvalue))
            print(self.path)
            print(moves)
            for move in moves:
                for step in self.path:
                    if step[1] == move and step[1] >= bestvalue:
                        best_move.append(move)
            print(best_move)
            i = 0
            if len(best_move) > 1:
                i = np.random.randint(0,len(best_move))
            picked_best = best_move[i]
            print(np.array(state))
            after = self.make_move(picked_best,cap,state,p)
            print('playing picked move')
            print(np.array(after[0]))
            return self.grid_to_world(picked_best,cap) #cap should be None




    def prune(self,alpha,beta,node,is_max,level): #based on pseudocode from https://www.hackerearth.com/blog/developers/minimax-algorithm-alpha-beta-pruning/
        '''Alpha beta pruning to optimize minimax'''
        #self.board.world_to_grid(node)
        #self.board.init_state = node
        val = 0
        #print('Level: '+str(level))
        self.is_game_over(node)
        #print('black: '+str(self.board.black_piece_count))
        #print('red: '+str(self.board.red_piece_count))
        if self.game_over or level > 50: #return utility of the node if terminal node
            if self.game_over == False:
                if is_max:
                    if self.board.baxter_color == 'black':
                        val = self.board.black_total
                    else:
                        val = self.board.red_total
                else:
                    if self.board.baxter_color == 'black':
                        val = self.board.black_total
                    else:
                        val = self.board.red_total
            else:
                # print('GAMEOVER')
                if self.winner == 'baxter': #returns count of baxter's pieces
                    if self.board.baxter_color == 'black':
                        val = self.board.black_total
                    else:
                        val = self.board.red_total
                elif self.winner == 'not_baxter': #return the count of not baxter's pieces
                    if self.board.not_baxter == 'black':
                        val = self.board.black_total
                    else:
                        val = self.board.red_total
                else:
                    val = 0 #if draw score is zero
            #self.path.append([move_taken,val])
            print('Terminal, returning '+str(val))
            return val

        if is_max == 'max': #baxter is maximizing
            #print(self.baxter_color)
            moves,cap,p = self.board.get_moves(node,self.board.baxter_color) #gets moves for current node
            #val = alpha
            count = 0
            # print(moves)
            init_state = copy.deepcopy(node)
            for move in moves:
                count += 1
                print('Max next move('+str(count)+' of '+str(len(moves))+') on level '+str(level))
                child,captured = self.make_move(move,cap,init_state,p)
                # print('child:\n '+str(np.array(child)))
                evaluate = self.prune(alpha,beta,child,'min',level-1)
                print('Utility of child node: '+str(evaluate))
                alpha = max(alpha,evaluate)
                self.path.append([self.board.baxter_color,move,alpha])
                #print('value: '+str(val))
                # print('alpha: '+str(alpha))
                # print('for Move '+str(count))
                '''Once terminal node reached, undo last move and set game_over and winner back to false/none'''
                init_state = self.undo_move(move,cap,captured,child,p)
                self.game_over = False #
                self.winner = None
                if beta <= alpha: #don't update alpha and prune if its greater than beta
                    print('Prune')
                    print('return alpha: '+str(alpha))
                    return alpha
                #alpha = max(alpha,val)
                print('returning val (max): '+str(alpha))
            return alpha #return maximum of all children

        else: #minimizing player
            moves, cap,p = self.board.get_moves(node,self.board.not_baxter) #gets moves for current node
            # print(moves)
            #val = beta
            init_state = copy.deepcopy(node)
            count = 0
            for move in moves:
                count += 1
                print('Min next move('+str(count)+' of '+str(len(moves))+') on level '+str(level))
                child,captured = self.make_move(move,cap,init_state,p)
                # print('child:\n '+str(np.array(child)))
                evaluate = self.prune(alpha,beta,child,'max',level+1)
                print('Utility of child node: '+str(evaluate))
                beta = min(evaluate,beta)
                #self.path.append([self.board.not_baxter,move,beta])
                # print('beta: '+str(beta))
                # print('for Move '+str(count))
                '''Once terminal node reached, undo last move and set game_over and winner back to false/none'''
                init_state = self.undo_move(move,cap,captured,child,p)
                self.game_over = False
                self.winner = None
                if beta <= alpha: #don't update alpha and prune if its greater than beta
                    print('Prune')
                    print('return beta:'+str(beta))
                    return beta
                #beta = min(beta,val)
                print('returning val (min): '+str(beta))
            return beta #return minimum of all children


class Board():
    '''Keeps track of board state and move generation. Initialized and called only in the CheckersAI class'''
    def __init__(self):#,state):
        self.red_piece_count = 0
        self.black_piece_count = 0
        self.baxter_color = None
        #self.init_state = state
        #self.state = state

    def flip_board(self,state):
        '''Args:
             state: a array representation of a board state.
           Returns:
             flip_state: the same board with the first index corresponding to the corner diagonally opposite.
            Flips the board so the board is in the perspective of whichever players turn it is'''
        flip_state = state[::-1]
        #for l in range(len(flip_state)): #this loop assumes baxter is dark
        #    flip_state[l] = flip_state[l][::-1]
        return flip_state

    def get_piece_count(self,state):
        self.red_piece_count = 0
        self.black_piece_count = 0
        self.red_total = 0
        self.black_total = 0
        red_king_count = 0
        black_king_count = 0
        for row in state:
            for col_ele in row:
                if col_ele < 0:
                    self.black_piece_count += 1
                    if col_ele == -2: black_king_count += 2
                elif col_ele > 0:
                    self.red_piece_count += 1
                    if col_ele == 2: red_king_count += 2
        self.red_total = self.red_piece_count + red_king_count
        self.black_total = self.black_piece_count + black_king_count

    def world_to_grid(self,list):
        '''recieve list[0,63], left to right, top to bottom from top left of board, index refers to grid square (inorder),elements are 'empty','color1',color2,'color1_king',color2_king'''
        '''Converts from computer vision input to grid array. Will be given an (x,y) position and color for that position.'''
        state = [[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]]
        r = 0
        c = 0
        if isinstance(list,str):
            list = list.split(' ')
        print(list)
        for ele in list:
            print('element: '+str(ele))
            if isinstance(ele,str):
                if ele == 'purple' or ele == 'black':
                    piece = -1
                elif ele == 'green' or ele == 'red':
                    piece = 1
                else:
                    piece = 0
                state[r][c] = piece
                if c == 7:
                    r += 1
                    c = 0
                else:
                    c += 1
            else:
                state[r][c] = ele
                #print(state)
                if c == 7:
                    r += 1
                    c = 0
                else:
                    c += 1
        #state = self.flip_board(state)
        if self.baxter_color == None:
            if state[-1][-2] == 1: #colors determined here !!
                self.baxter_color = 'red'
                self.not_baxter = 'black'
            elif state[-1][-2] == -1:
                self.baxter_color = 'black'
                self.not_baxter = 'red'
        #print(self.baxter_color)
        self.init_state = state
        self.get_piece_count(state)
        #print(state)
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
            if state[r][c] == self.p:
                step = [[self.sgn*1,1],[self.sgn*1,-1]]
            #elif state[r][c] == self.p and self.p == 1:
            #    step = [[1,1],[1,-1]]

            if state[r][c] == self.p*2:
                step = [[1,1],[1,-1],[-1,1],[-1,-1]]
                #print('King')
            for s in step: #take a step in valid directions
                stepc = s[1]
                stepr = s[0]
                # print(moves)
                if c+stepc >= 0 and c+stepc <= 7 and r+stepr >= 0 and r+stepr <= 7: # Checks that the step won't take piece out of bounds
                    '''Single Step'''
                    if state[r+stepr][c+stepc] == 0: #if the diagonal square is empty, add step to list of legal moves
                        moves.append([[r,c],[r+stepr,c+stepc]])
                        # print(True)
                    else: #If not empty, try to make a jump
                        # print('jump')
                        move, cap, cap_success = can_jump(r,c,stepr,stepc,state) #single jump
                        '''Double jump'''
                        if cap_success: #check for double jump
                            capture = [cap] #store captured piece position if successful jump
                            moves = [move]
                            for s2 in step: #check all possible step directions
                                stepc2 = s2[1]
                                stepr2 = s2[0]
                                parent = move[0] #save start position from first jump
                                temp, dcap,cap_success = can_jump(move[1][0],move[1][1],stepr2,stepc2,state) #double jump
                                if cap_success:
                                    moves[0][1] = temp[1] #replace previously saved end position with end position after double jump
                                    capture.append([parent,dcap[1]])

                            return moves, capture

            return moves, capture



        #currently assuming list
        self.get_piece_count(state)
        # print(self.red_piece_count)
        moves = []
        temp = []
        capture = []
        steps = [[1,-1],[1,1]]
        self.step_r = 1
        self.sgn = 1
        if player.lower() == self.baxter_color: #state is always read from baxter's perspective! wrong:in starting with dark color first since black player goes first
            self.sgn = -1
            if self.baxter_color == 'black': #black is -1
                self.p = -1 #baxter starts from the bottom
                self.not_p = 1
            else:
            #state = flip_board(state)
                self.p = 1 #Red is +1
                self.not_p = -1
        else:
            if self.not_baxter == 'black': #black is -1
                self.p = -1 #baxter starts from the bottom
                self.not_p = 1
            else:
            #state = flip_board(state)
                self.p = 1 #Red is +1
                self.not_p = -1
        #cycle through grid cells
        for r in range(8):
            for c in range(8):
                # print(r,c)
                mK,capK = can_step(r,c,state)

                temp_moves = []
                # print(mK)
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
            return capture_move, capture,self.p

        return moves,capture,self.p
