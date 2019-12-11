import numpy as np
import copy
class CheckersAI():
    '''Keeps track of game progress and uses Minimax with alpha beta pruning to find best move for the Baxter robot'''
    def __init__(self):
        '''Initialize CheckersAI class variables and creates Board class object'''
        self.game_over = False
        self.whos_turn = 'black' #black goes first
        self.noCapture = 0
        self.winner = None
        self.board = Board()

    def give_command(self,state_list):#,color(used for testing)):
        '''ARGS:
                state_list:string that stores the positions of the pieces on the board, output from CV node
        Return:
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
        self.board.prev_state = after_move[0]
        return self.grid_to_world([start, goal], captured)

    def switch_turn(self):
        '''Sets variable from whichever players turn it is to the other player'''
        if self.whos_turn == 'Red':
            self.whos_turn = 'Black'
        else:
            self.whos_turn = 'Red'

    def is_game_over(self,state):
        '''ARGS:
            state: board state array
        Takes board state and checks if the game is over by checking number of pieces left on the board, or whoever's turn it
        is has no moves. Assigns winner variable to whoever won
        '''
        self.board.get_piece_count(state)
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
        elif self.board.get_moves(state,self.whos_turn) == None: #no legal moves.
            self.switch_turn()
            if self.whos_turn == self.board.baxter_color:
                self.winner = 'baxter'
            else:
                self.winner = 'not_baxter'
            self.game_over = True

    def grid_to_world(self,move,captured):
        '''ARGS:
            move: list of start and goal postion ([[0,0],[2,2]])
            captured: list of start and captured piece locations ([[0,0],[1,1]])
        Return:
            movelist: list of indeices corresponding to grid coordinates
        Converts moves from grid coordinates to index of 1-d list refering to board positions
        assumes left to right, bottom to top from baxter's perspective'''
        movelist = [move[0][0]*8+move[0][1],move[1][0]*8+move[1][1]]
        for cap in captured:
            if cap[1][0]*8+cap[1][1] not in movelist:
                movelist.append(cap[1][0]*8+cap[1][1])
        print('Move list: '+str(movelist))
        return movelist

    def make_move(self,move,cap,state,p,is_ai=False):
        '''ARGS:
            move: [[start,goal]], position of the move to be made
            cap: [[start,capture]], position of any captures from that start position
            state: the current board state array
            p: +/-1 to denote which color's turn it is
            is_ai: (bool) whether function is being used in context of minimax(forward simulation) or actaully making the move
        Return:
            state: updated board array
            captured: list of positions of captured pieces
        Make an updated array of board state after legal move, starting from given state.'''
        if len(state) > 8:
            state = self.board.world_to_grid(state)
        K = 1
        if self.board.baxter_color == 'black':
            baxter = -1
            not_bax = 1
        else:
            baxter = 1
            not_bax = -1
        if p == baxter and move[1][0] == 0 or p == not_bax and move[1][0] == 7 or state[move[0][0]][move[0][1]] == p*2:
            K = 2
            if not is_ai: #only update the actual move, not during minimax
                self.board.bax_king_list.append(move[1])
                print(self.board.bax_king_list)
        state[move[0][0]][move[0][1]] = 0
        state[move[1][0]][move[1][1]] = K*p
        captured = []
        if len(cap) != 0 and cap[0][0] == move[0]:
            for piece in cap:
                captured.append(state[piece[1][0]][piece[1][1]])
                state[piece[1][0]][piece[1][1]] = 0
        self.is_game_over(state)
        return state,captured

    def undo_move(self,move,cap,cap_piece,state,player):
        '''ARGS:
            move: [[start,goal]], position of the move to be made
            cap: [[start,capture]], position of any captures from that start position
            cap_piece: +/-1 to denote which color of the captured piece
            state: the current board state array
            player: +/-1 to denote which color's turn it is
        Return:
            state: updated board array
        Returns board array to state before a move was made'''
        K = 1
        if state[move[1][0]][move[1][1]] == player*2: #only have to worry about piece before move
            K = 2
        state[move[1][0]][move[1][1]] = 0
        state[move[0][0]][move[0][1]] = K*player #should still be the last players value
        if len(cap) != 0 and cap[0][0] == move[0]:
            for i in range(len(cap)):
                state[cap[i][1][0]][cap[i][1][1]] = cap_piece[i]
        return state

    def minimax(self,state_list):
        '''ARGS:
            state_list: string containinig board occpancy at each index(board square) from top left to bottom
        Return:
            movelist: list of board indeices (0-63) from top left to bottom
        Runs Minimax algorithm to get best moves for baxter. Converts input to board array and checks if there are any captures, which are passed immediatly. If no captures,
        calls the prune function(alpha beta pruning) to get the best move value. Compares that value to the list of moves from the initial state, and saves a list of the legal moves
        that match the bestvalue. If there are more than one legal move with best value, randomly select one. Convert this picked move to the movelist and return it.
        '''
        state = self.board.world_to_grid(state_list)
        best_move = []
        moves,cap,p = self.board.get_moves(state,self.board.baxter_color)
        # print(moves)
        # self.val_list = []
        if len(moves) == 1:
            print('Picked move: '+str(moves[0]))
            confirmed = raw_input('Play move(y/n)?')
            if confirmed == 'n':
                return [None]
            after = self.make_move(moves[0],cap,state,p)
            self.board.prev_state = after[0]
            print(np.array(after[0]))
            return self.grid_to_world(moves[0],cap) #captured moves are forced
        else:
            self.path = []
            bestvalue = self.prune(float('-inf'),float('+inf'),state,'max',0)
            # bestvalue = self.basic_search(state,'min',0)
            print('Best_value: '+str(bestvalue))
            exists = False
            for move in moves:
                for step in self.path:
                    if step[0] == move:
                        if step[1] == bestvalue:
                            if step[0] not in best_move:
                                best_move.append(step[0])
                        elif step[1] > bestvalue:
                            best_move = [step[0]]
                            break
            # print('Best move: '+str(best_move))
            i = 0
            if len(best_move) > 1:
                i = np.random.randint(0,len(best_move))
            picked_best = best_move[i]
            print('Picked move: '+str(picked_best))
            confirmed = raw_input('Play move(y/n)?')
            if confirmed == 'n':
                return [None]
            print(np.array(state))
            after = self.make_move(picked_best,cap,state,p)
            self.board.prev_state = after[0]
            print(np.array(after[0]))

            return self.grid_to_world(picked_best,cap) #cap should be None

    def score_board(self,state):
        '''ARGS:
            state: board state to score
        Return:
            val: score of the board
        Calculates the score of the board. conditions change based on if it results in a game over or just minimum layer'''
        if self.game_over == False:

            if self.board.baxter_color == 'black':
                val = 4*(self.board.black_total - self.board.red_total)
            else:
                val = 4*(self.board.red_total - self.board.black_total)
        else:
            # print('GAMEOVER')
            if self.board.baxter_color == 'black':
                val = 4*(self.board.black_total - self.board.red_total)+10 #if baxter won, val will be positive
            else:
                val = 4*(self.board.red_total - self.board.black_total)+10 #if baxter lost, val will be negative (more enemy pieces)

        return val

    def basic_search(self,node,is_max,level): #Minimax with no optimization (alpha beta pruning). works, but scoreing needs work
        '''ARGS:
            node: current state array
            is_max: string denoting whether its the maximizing or minimizing players turn
            level: current depth of tree
        Return:
            val/max_v/min: terminal/mamximum/minimum score
        Basic minimax, no pruning, where it applys moves to the initial state and returns the score from the terminal nodes back up the layers
        taking either the minimum or maximum score depending on the layer
        '''
        val = 0
        self.is_game_over(node)
        if self.game_over or level > 4: #return utility of the node if terminal node
            return self.score_board(node)

        if is_max == 'max': #baxter is maximizing
            moves,cap,p = self.board.get_moves(node,self.board.baxter_color) #gets moves for current node
            max_v = float("-inf")
            count = 0
            init_state = copy.deepcopy(node)
            for move in moves:
                count += 1
                # print('Max next move('+str(count)+' of '+str(len(moves))+') on level '+str(level))
                child,captured = self.make_move(move,cap,init_state,p,is_ai=True)
                # print('child:\n '+str(np.array(child)))
                evaluate = self.basic_search(child,'min',level+1)
                # print("Utility of Max's child node: "+str(evaluate)+" vs. current Max value: "+str(max_v))
                max_v = max(max_v,evaluate)
                # print('Updated Max Value: '+str(max_v))

                self.path.append([move,max_v])
                '''Once terminal node reached, undo last move and set game_over and winner back to false/none'''
                init_state = self.undo_move(move,cap,captured,child,p)
                self.game_over = False #
                self.winner = None
            return max_v #return maximum of all children

        else: #minimizing player
            moves, cap,p = self.board.get_moves(node,self.board.not_baxter) #gets moves for current node
            min_v = float("+inf")
            init_state = copy.deepcopy(node)
            count = 0
            for move in moves:
                count += 1
                # print('Min next move('+str(count)+' of '+str(len(moves))+') on level '+str(level))
                child,captured = self.make_move(move,cap,init_state,p,is_ai=True)
                # print('child:\n '+str(np.array(child)))
                evaluate = self.basic_search(child,'max',level+1)
                # print("Utility of Min's child node: "+str(evaluate)+" vs. current Min value: "+str(min_v))
                min_v = min(min_v,evaluate)
                # print('Updated Min Value: '+str(min_v))
                '''Once terminal node reached, undo last move and set game_over and winner back to false/none'''
                init_state = self.undo_move(move,cap,captured,child,p)
                self.game_over = False
                self.winner = None
            return min_v #return minimum of all children


    def prune(self,alpha,beta,node,is_max,level): #based on pseudocode from https://www.hackerearth.com/blog/developers/minimax-algorithm-alpha-beta-pruning/
        '''ARGS:
            alpha: alpha value for this node
            beta: beta value for this node
            node: board state array
            is_max: string denoting whether its the maximizing or minimizing players turn
            level: current depth of tree
        Return:
            val/max_v/min: terminal/mamximum/minimum score
        Expansion on the basic_search function. Uses alpha beta pruning to optimize minimax algorithm'''
        val = 0
        self.is_game_over(node)
        if self.game_over or level > 9: #return utility of the node if terminal node
            return self.score_board(node)

        if is_max == 'max': #baxter is maximizing
            moves,cap,p = self.board.get_moves(node,self.board.baxter_color) #gets moves for current node
            max_v = float("-inf")
            count = 0
            init_state = copy.deepcopy(node)

            for move in moves:
                # print('Alpha: '+str(alpha))
                count += 1
                # print('Max next move('+str(count)+' of '+str(len(moves))+') on level '+str(level))
                child,captured = self.make_move(move,cap,init_state,p,is_ai=True)
                # print('child:\n '+str(np.array(child)))
                evaluate = self.prune(alpha,beta,child,'min',level+1)
                # print("Utility of Max's child node: "+str(evaluate)+" vs. current Max value: "+str(max_v))

                max_v = max(max_v,evaluate)
                # print('Updated Max Value: '+str(max_v))
                # print('current Alpha: '+str(alpha))
                alpha = max(alpha,evaluate)
                # print('Updated Alpha: '+str(alpha))
                if level == 0:
                    self.path.append([move,max_v])

                '''Once terminal node reached, undo last move and set game_over and winner back to false/none'''
                init_state = self.undo_move(move,cap,captured,child,p)
                self.game_over = False #
                self.winner = None
                if beta <= alpha: #Prune condition
                    # print('Prune the rest of level '+str(level))
                    break
            return max_v #return maximum of all children

        else: #minimizing player
            moves, cap,p = self.board.get_moves(node,self.board.not_baxter) #gets moves for current node
            min_v = float("+inf")
            init_state = copy.deepcopy(node)
            count = 0
            for move in moves:
                count += 1
                # print('Min next move('+str(count)+' of '+str(len(moves))+') on level '+str(level))
                child,captured = self.make_move(move,cap,init_state,p,is_ai=True)
                # print('child:\n '+str(np.array(child)))
                evaluate = -self.prune(alpha,beta,child,'max',level+1)
                # print("Utility of Min's child node: "+str(evaluate)+" vs. current Min value: "+str(min_v))
                min_v = min(min_v,evaluate)
                # print('Updated Min Value: '+str(min_v))
                # print('current Beta: '+str(beta))
                beta = min(beta,evaluate)
                # print('Updated Beta: '+str(beta))

                '''Once terminal node reached, undo last move and set game_over and winner back to false/none'''
                init_state = self.undo_move(move,cap,captured,child,p)
                self.game_over = False
                self.winner = None

                if beta <= alpha: #Prune condition
                    # print('Prune the rest of level '+str(level))
                    break
            return min_v #return minimum of all children


class Board():
    '''Keeps track of board state and move generation. Initialized and called only in the CheckersAI class'''
    def __init__(self):#,state):
        '''Initialize Board class variables'''
        self.red_piece_count = 0
        self.black_piece_count = 0
        self.baxter_color = None
        self.prev_state = None
        self.bax_king_list = []
        self.enemy_king_list = []


    def get_piece_count(self,state):
        '''ARGS:
            state: board state array
        From a given board state counts up number of pieces for each side and stores that count in class attribute. Function
        also used for assigning scores for the board state that are used in the minimax algorithm'''
        self.red_piece_count = 0
        self.black_piece_count = 0
        self.red_total = 0
        self.black_total = 0
        red_king_count = 0
        black_king_count = 0
        advance_b = 0
        advance_r = 0
        row_cnt = 0
        col_cnt = 0
        for row in state:
            for col_ele in row:
                if col_ele < 0:
                    self.black_piece_count += 1
                    if col_ele == -2: black_king_count += 4
                    if row_cnt > 3: advance_b += 1
                    if col_cnt > 1 and col_cnt < 6: advance_b += 1
                elif col_ele > 0:
                    self.red_piece_count += 1
                    if col_ele == 2: red_king_count += 4
                    if row_cnt > 3: advance_r += 1
                    if col_cnt > 1 and col_cnt < 6: advance_r += 1
                col_cnt += 1
            row_cnt += 1
        self.red_total = self.red_piece_count + red_king_count + advance_b
        self.black_total = self.black_piece_count + black_king_count + advance_r

    def world_to_grid(self,list):
        '''ARGS:
            list: [0,63], left to right, top to bottom from top left of board, index refers to grid square (inorder),elements are 'empty','color1','color2'
        Return:
            state: current board state array from computer vision
        Converts from computer vision input to grid array. Will be given an (x,y) position and color for that position. Updates position of Kings on the Board
        and keeps track of new kings from the human player's turn'''
        state = [[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]]
        r = 0
        c = 0
        if isinstance(list,str):
            list = list.split(' ')
        king_move = 1
        # print(len(list))
        try:
            for ele in list:
                # print('element: '+str(ele))
                if isinstance(ele,str):
                    if ele == 'purple' or ele == 'black':
                        piece = -1
                    elif ele == 'green' or ele == 'red':
                        piece = 1
                    else:
                        piece = 0
                    state[r][c] = piece
                    # print(state)

                    if self.prev_state != None:
                        if self.baxter_color == 'black':
                            baxter = -1
                            not_bax = 1
                        else:
                            baxter = 1
                            not_bax = -1
                        if state[r][c] != self.prev_state[r][c]: #board state changed
                            for king in self.enemy_king_list: #loop through enemy kings
                                print(king)
                                if state[r][c] == 0 and king == [r,c]: #if a king is no longer where it was
                                    king_move == True # we know its a king's move
                                    print('king_move')
                                    try:
                                        self.enemy_king_list.remove([r,c]) #the old index for the king is removed
                                        print('enemy: '+str(self.enemy_king_list))
                                    except ValueError:
                                        print('ValueError, not on list')

                            if state[r][c] != 0 and state[r][c] == not_bax: #only humans piece should be moved.
                                state[r][c] *= 2
                                if [r,c] not in self.enemy_king_list:
                                    self.enemy_king_list.append([r,c]) #add new location of king to list
                                # print('king_move '+str(self.enemy_king_list))

                            for king in self.bax_king_list: #loop through baxter's kings
                                if [r,c] in self.bax_king_list and state[r][c] != 0: #king on baxter's list, and is still there
                                    state[r][c] *= 2 # update value
                                    # print('baxters kings updated')
                                elif [r,c] in self.bax_king_list and state[r][c] == 0: #if no longer there, jumped
                                    try:
                                        # print('remove king from baxters list')
                                        self.bax_king_list.remove([r,c]) #remove from king list
                                    except ValueError:
                                        print('ValueError, not on list')


                    if c == 7:
                        r += 1
                        c = 0
                    else:
                        c += 1
                else: #used for testing
                    state[r][c] = ele
                    if c == 7:
                        r += 1
                        c = 0
                    else:
                        c += 1
        except IndexError:
            print('IndexError: length of list is '+str(len(list)))
        # print(self.bax_king_list)
        # print(self.enemy_king_list)
        if self.baxter_color == None:
            if state[-1][-2] == 1: #colors determined here !!
                self.baxter_color = 'red'
                self.not_baxter = 'black'
            elif state[-1][-2] == -1:
                self.baxter_color = 'black'
                self.not_baxter = 'red'
        self.init_state = state
        self.get_piece_count(state)
        # print(np.array(state))
        return state



    def get_moves(self,state,player):
        '''ARGS:
            state: current board state
            player: current players color, used to check moves for only that players moves
        Return:
            moves: list of all legal moves and captured pieces as tuple pairs (Ex. [[start],[goal]])
            capture: list of any captured pieces (Ex.[[start],[captured_position]])
            p: +/- 1 representing the player's color
        Determines player's color and uses that to determine which direction across the board to move. For example baxter pawns
        move from row 7 of the array to row 0, and the other player's pawns move the opposite directions. Loops through the board state
        to find pieces and their possible moves. Jumps are forced so only the jumping move is returned. '''
        def can_jump(row,col,step_r,step_c,state):
            '''ARGS:
                row: current row
                col: current column
                step_r: directions to step row-wise
                step_c: directions to step column-wise
                state: board state array
               Returns:
                move: (start, goal) pair (if any) of legal moves
                captured: (start,captured piece position) pair of any captured pieces
                cap: (bool) whether any pieces were captured or not
            Checks if step in given direction keeps piece in bounds, if it does, checks if there is a place to jump, and that the jumped piece is the other player's
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
                 state: board state array
            Return:
                 moves: list of (start, goal) pairs (if any) of legal moves
                 capture: list of (start,captured piece position) pairs of any captured pieces
            Checks if a step is possible is each possible direction for a type of piece. if step is not possible, checks for jump.
            if jump was successful, checks for possible double jumps. Jumps are forced so any jumps overwrites any existing moves and does not
            look for any additional moves.'''
            moves = []
            capture = []
            step = []
            #set possible steps based on whether a pawn or king and which player
            if state[r][c] == self.p:
                step = [[self.sgn*1,1],[self.sgn*1,-1]]

            if state[r][c] == self.p*2:
                step = [[1,1],[1,-1],[-1,1],[-1,-1]]
            for s in step: #take a step in valid directions
                stepc = s[1]
                stepr = s[0]
                if c+stepc >= 0 and c+stepc <= 7 and r+stepr >= 0 and r+stepr <= 7: # Checks that the step won't take piece out of bounds
                    '''Single Step'''
                    if state[r+stepr][c+stepc] == 0: #if the diagonal square is empty, add step to list of legal moves
                        moves.append([[r,c],[r+stepr,c+stepc]])
                    else: #If not empty, try to make a jump
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


        self.get_piece_count(state)
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
                self.p = 1 #Red is +1
                self.not_p = -1
        else:
            if self.not_baxter == 'black': #black is -1
                self.p = -1 #baxter starts from the bottom
                self.not_p = 1
            else:
                self.p = 1 #Red is +1
                self.not_p = -1
        #cycle through grid cells
        for r in range(8):
            for c in range(8):
                mK,capK = can_step(r,c,state)
                temp_moves = []
                for i in range(len(mK)):
                    if len(mK[i]) != 0:
                        temp_moves.append(mK[i])
                        moves.append(mK[i])
                for i in range(len(capK)):
                    if len(capK[i]) != 0:
                        capture.append(capK[i])
                        capture_move = temp_moves
                        #return temp_moves, capture
        if len(capture) != 0:
            return capture_move, capture,self.p

        return moves,capture,self.p
