


class CheckersAI():
    '''Keeps track of game progress and uses Minimax with alpha beta pruning to find best move for the Baxter robot'''
    def __init__(self):
        self.game_over = False
        self.whos_turn = 'Red'
        self.noCaptureCounter = 0
        self.winner = None
        self.alpha = float('-inf')
        self.beta = float('+inf')

    def give_command(self,state):
        valid = False
        #board = Board(state)
        #moves = board.get_moves()
        while not valid:
            start_in = raw_input('Enter position of peice you want to move(Ex. 3,3 = third column and third row from bottom left corner): ')
            goal_in = raw_input('Enter where you want to move the peice to: ')
            start = start_in.split(',')
            goal = goal_in.split(',')
            for i in range(len(start)):
                try:
                    start[i] = int(start[i])
                    goal[i] = int(goal[i])
                    #for move in moves:
                        #check if legal move
                    valid = True
                except ValueError:
                    print('\nEntered move is not in valid format. Please enter two integers seperated by a comma\n')
                    valid = False

        return start, goal

    def switch_turn(self):

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
        self.state = state #should be array of peices
        self.red_piece_count = 12
        self.black_piece_count = 12
        self.R_steps = [[-1,1],[1,1]] #bottom up left to right
        self.B_steps = [[-1,-1],[1,-1]] #top down left to right



    def get_moves(self):
        for row in self.state:
            for cell in row:
                if cell == 'R':
                    for step in self.R_steps:

        pass
