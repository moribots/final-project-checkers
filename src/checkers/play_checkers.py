
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
                board: Board object that stores the positions of the peices on the board, used to check if the move is legal
                color: designates what color the player is, used for get_moves
           Returns:
                start: Grid position of the peice to move
                goal: Grid position of where to move the peice to
                TODO: captured: returns position of any captured peices'''
        valid = False
        captured = None
        moves = board.get_moves(color)
        while not valid:
            start_in = raw_input('Enter position of peice you want to move or q to quit: ')
            if start_in == 'q':
                print('Player quit :(')
                return None,None,None
            goal_in = raw_input('Enter where you want to move the peice to: ')
            start = start_in.split(',')
            goal = goal_in.split(',')
            for i in range(len(start)):
                try:
                    start[i] = int(start[i])-1 #convert o python counting convention
                    goal[i] = int(goal[i])-1 #convert o python counting convention
                except ValueError:
                    print('\nEntered move is not in valid format. Please enter two integers seperated by a comma\n')
                    valid = False
            for move in moves:
                #check if legal move
                if [start,goal] == move:
                    print('Moving...')
                    valid = True
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

        def can_step(r,c,state,player,not_player,stepr,stepc):
            move = None
            if state[r][c] == player:# checks that its players peice
                '''Single Step'''
                if state[r+stepr][c+stepc] == 0: #if the diagonal square is empty
                    move = [[r,c],[r+stepr,c+stepc]]
                elif r < 5 and c < 5: #if not enough space to make a jump
                    #otherwise make a jump if it is the opponents peice
                    if state[r+(2*stepr)][c+(2*stepc)] == 0 and state[r+stepr][c+stepc] == not_player:
                        #capture condition met
                        move = [[r,c],[r+(2*stepr),c+(2*stepc)]]
                        capture = [[r,c],[r+(stepr),c+(stepc)]]
                        if p == -1:
                            self.black_piece_count -= 1
                        else:
                            self.red_piece_count -= 1
            return move

        def flip_board(state):
            flip_state = state[::-1]
            for l in range(len(flip_state)):
                #print(l)
                flip_state[l] = flip_state[l][::-1]
            return flip_state

        #currently assuming list
        moves = []
        temp = []
        steps = [[1,-1],[1,1]]
        #print(self.state[0])
        if player == 'Black': #state is always read in starting with dark color first
            #Black is -1
            p = -1
            not_p = 1
        else:
            self.state = flip_board(self.state)
            p = 1 #Red is +1
            not_p = -1
        #cycle through grid cells
        for r in range(8):
            for c in range(8):
                #print(r,c)
                if r == 7:#only kings can move
                    if self.state[r][c] == p*2 and p*2 != not_p*2:
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
                        temp = can_step(r,c,self.state,p,not_p,steps[1][0],steps[1][1])
                        if temp != None:
                            moves.append(temp)
                    elif c == 7:
                        temp = can_step(r,c,self.state,p,not_p,steps[0][0],steps[0][1])
                        if temp != None:
                            moves.append(temp)
                    else:
                        for step in steps:
                            temp = can_step(r,c,self.state,p,not_p,step[0],step[1])
                            if temp != None:
                                moves.append(temp)
        #print(moves)
        return(moves)
