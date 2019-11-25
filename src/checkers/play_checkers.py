
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
        self.r_steps = [[-1,1],[1,1]] #bottom up left to right
        self.b_steps = [[-1,-1],[1,-1]] #top down left to right
        self.king_steps = [[-1,-1],[1,-1],[[-1,1],[1,1]]]
        self.state = state
        #self.type = [('r',self.r_steps),('c',self.b_steps),('R',self.king_steps),('B',,self.king_steps)]

    def world_to_grid(self):
        '''Converts from computer vision input to grid array. Will be given an (x,y) position and color for that position.'''
        pass

    def get_moves(self,player):

        def can_jump(row,col,stepc):#,stepc):
            captured = []
            move = []
            cap = False
            if col > 5 and stepc > 0: #make sure pawn doesn't try to jump off board
                stepc = -1
            if self.state[row+(2*self.step_r)][col+(2*stepc)] == 0 and self.state[row+self.step_r][col+stepc] == self.not_p:
                #capture condition met
                move = [[row,col],[row+(2*self.step_r),col+(2*stepc)]]
                captured = [[row,col],[row+(self.step_r),col+(stepc)]]
                cap = True
                if self.p == -1:
                    self.black_piece_count -= 1
                else:
                    self.red_piece_count -= 1
            return move,captured,cap

        def can_step(r,c,step):
        ''' Args:
                r: current row index
                c: current column index
                step: either 1,-1, or both, states which direction side to side the piece can move diagonally
            Returns:
                moves: list of (start, goal) pairs (if any) of legal moves
                capture: list of (start,captured piece position) pairs of any captured pieces'''
            moves = []
            capture = []

            for s in step: #try to take a step in valid directions
                stepc = self.step_r*s
                #print(stepc)
                if self.state[r][c] == self.p and c <= 6:# checks that its players piece
                    '''Single Step'''

                    if self.state[r+self.step_r][c+stepc] == 0: #if the diagonal square is empty, add to list of legal moves
                        moves.append([[r,c],[r+self.step_r,c+stepc]])
                    elif r <= 6 : #if enough space to make a jump
                        #make a jump if it is the opponents piece
                        move, cap, cap_success = can_jump(r,c,stepc) #single jump
                        moves = [move]
                        if len(cap) != 0:
                            capture = [cap]
                        '''Double jump'''
                        if cap_success and r < 3:#check for double jump
                            for s2 in step:
                                stepc2 = self.step_r*s2
                                parent = move[0] #save start position from first jump
                                temp, dcap,cap_success = can_jump(move[1][0],move[1][1],stepc2) #double jump
                                if cap_success:
                                    moves.append([parent,temp[1]]) #return parent from first jump and goal from second jump
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
        if player == 'Black': #state is always read in starting with dark color first
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
                if r == 7:#only kings can move
                    if self.state[r][c] == self.p*2 and self.p*2 != self.not_p*2:
                        for step in self.king_steps:
                            try:
                                print([[self.state[r][c]],[self.state[r+step[0][0]][c+step[0][1]]]])
                                temp.append([[self.state[r][c]],[self.state[r+step[0][0]][c+step[0][1]]]])
                                moves.append([[r,c],[r+step[0][0],c+step[0][1]]])
                                print('Move King')
                            except IndexError:
                                print('out of bounds')
                else:
                    if c == 0: #can only move toward center of board
                        m,cap = can_step(r,c,[1])
                        for i in range(len(m)):
                            if len(m[i]) != 0:
                                moves.append(m[i])
                        for i in range(len(cap)):
                            if len(cap[i]) != 0:
                                capture.append(cap[i])

                    elif c == 7: #can only move toward center of board
                        m,cap = can_step(r,c,[-1])
                        for i in range(len(m)):
                            if len(m[i]) != 0:
                                moves.append(m[i])
                        for i in range(len(cap)):
                            if len(cap[i]) != 0:
                                capture.append(cap[i])
                    else: #can move diagonally in either direction side to side
                        m,cap = can_step(r,c,[1,-1])
                        for i in range(len(m)):
                            if len(m[i]) != 0:
                                moves.append(m[i])
                        for i in range(len(cap)):
                            if len(cap[i]) != 0:
                                capture.append(cap[i])
        #print(moves)
        return moves, capture
