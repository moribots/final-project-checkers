#!/usr/bin/env python

# import rospy
from checkers.play_checkers import CheckersAI, Board
import unittest
import numpy as np

class TestAI(unittest.TestCase):
    '''Check that the corret number of moves is being returned. Success indicates that the move generation is
    working '''
    def test_move_gen(self):
        '''Checks all pawn moves are accounted for, no jumps.'''
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_result = 7
        test_state = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  1,  0,  0,  0,  1,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  0,  0]]
        moves, cap,p = ai.board.get_moves(test_state,ai.board.not_baxter)
        self.assertEquals(len(moves),test_result)

    def test_one_jump(self):
        '''Checks that pawns take single jump, and only that move is returned'''
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_result = 1
        test_state = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  1],
                      [ 0,  0,  1,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  1,  0]]
        moves, cap,p = ai.board.get_moves(test_state,ai.board.not_baxter)
        self.assertEquals(len(moves),test_result)

    def test_double_jump(self):
        '''Checks that pawns take double jump, and only that move is returned'''
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_result = 1
        test_state = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  1],
                      [ 0,  0,  1,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  0,  0]]
        moves, cap,p = ai.board.get_moves(test_state,ai.board.not_baxter)
        self.assertEquals(len(moves),test_result)

    def test_king_move_gen(self):
        '''Checks all pawn and king moves are accounted for, no jumps.'''
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_result = 7
        test_state = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -2,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  1],
                      [ 0,  0,  1,  0,  1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  0,  0]]
        moves, cap,p= ai.board.get_moves(test_state,ai.board.not_baxter)
        self.assertEquals(len(moves),test_result)

    def test_king_one_jump(self):
        '''Checks that kings take single jump, and only that move is returned'''
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_result = 1
        test_state = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -2,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  1],
                      [ 0,  0,  1,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  1,  0]]
        moves, cap,p = ai.board.get_moves(test_state,ai.board.not_baxter)
        self.assertEquals(len(moves),test_result)

    def test_king_double_jump(self):
        '''Checks that kings take double jump, and only that move is returned'''
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_result = 1
        test_state = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -2,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -2,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  1],
                      [ 0,  0,  1,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  0,  0]]
        moves, cap,p = ai.board.get_moves(test_state,ai.board.not_baxter)
        self.assertEquals(len(moves),test_result)

    def test_read_state_pawns(self):
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_state = 'empty empty empty empty empty empty empty empty purple empty purple empty purple empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty purple empty empty empty empty empty empty empty empty empty empty empty green empty empty empty green empty empty empty empty empty empty green empty empty green empty empty empty empty empty empty empty '
        test_result = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  1,  0,  0,  0,  1,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  0,  0]]
        self.assertEquals(ai.board.world_to_grid(test_state),test_result)

    def test_read_state_pawns(self):
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        test_state = 'empty empty empty empty empty empty empty empty purple empty purple empty purple empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty purple empty empty empty empty empty empty empty empty empty empty empty green empty empty empty green empty empty empty empty empty empty green empty empty green empty empty empty empty empty empty empty '
        test_result = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  1,  0,  0,  0,  1,  0],
                      [ 0,  0,  0,  0,  0,  1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  0,  0]]
        self.assertEquals(ai.board.world_to_grid(test_state),test_result)

    def test_new_enemy_king(self):
        ai = CheckersAI()
        ai.board.baxter_color = 'red'
        ai.board.not_baxter = 'black'
        ai.board.prev_state = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  1,  0,  0,  0,  1,  0],
                      [ 0,  0,  0,  0,  0,  -1,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  0,  0]]
        test_state = 'empty empty empty empty empty empty empty empty purple empty purple empty purple empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty empty purple empty empty empty empty empty empty empty empty empty empty empty green empty empty empty green empty empty empty empty empty empty green empty empty green empty empty empty empty empty empty empty '
        test_result = [[ 0,  0,  0,  0,  0,  0,  0,  0],
                      [-1,  0, -1,  0, -1,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  0,  0,  0,  0, -1,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 0,  0,  1,  0,  0,  0,  1,  0],
                      [ 0,  0,  0,  0,  0,  0,  0,  0],
                      [ 1,  0,  0,  0,  0,  0,  -2,  0]]
        self.assertEquals(ai.board.world_to_grid(test_state),test_result)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('checkers','check_move_gen',TestAI)
