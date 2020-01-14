## Multi-agent mixed coop-comp setting RL
## Setting up reward functions, states and MARL environment

__author__ = 'philippe, edited for MARL by meghdeep'
import numpy as np
from tkinter import *
master = Tk()


triangle_size = 0.1
cell_score_min = -0.5
cell_score_max = 0.5
Width = 80
(x, y) = (5,5)
actions = ["up", "down", "left", "right"]

board = Canvas(master, width=x*Width, height=y*Width)
agents = [(4, 0),(0,4)]
score = [1,1]
restart = False
walk_reward = -0.04

#walls = [(1, 1), (1, 2), (2, 1), (2, 2), (3,4), (3,3), (0,0)]
walls=[(2,2)]
specials = [ (4, 4, "green", 2), (0, 0, "red", -2)]
cell_scores = {}

def create_triangle(i, j, action):
    if action == actions[0]:
        return board.create_polygon((i+0.5-triangle_size)*Width, (j+triangle_size)*Width,
                                    (i+0.5+triangle_size)*Width, (j+triangle_size)*Width,
                                    (i+0.5)*Width, j*Width,
                                    fill="white", width=1)
    elif action == actions[1]:
        return board.create_polygon((i+0.5-triangle_size)*Width, (j+1-triangle_size)*Width,
                                    (i+0.5+triangle_size)*Width, (j+1-triangle_size)*Width,
                                    (i+0.5)*Width, (j+1)*Width,
                                    fill="white", width=1)
    elif action == actions[2]:
        return board.create_polygon((i+triangle_size)*Width, (j+0.5-triangle_size)*Width,
                                    (i+triangle_size)*Width, (j+0.5+triangle_size)*Width,
                                    i*Width, (j+0.5)*Width,
                                    fill="white", width=1)
    elif action == actions[3]:
        return board.create_polygon((i+1-triangle_size)*Width, (j+0.5-triangle_size)*Width,
                                    (i+1-triangle_size)*Width, (j+0.5+triangle_size)*Width,
                                    (i+1)*Width, (j+0.5)*Width,
                                    fill="white", width=1)


def render_grid():
    global specials, walls, Width, x, y, agents
    for i in range(x):
        for j in range(y):
            board.create_rectangle(i*Width, j*Width, (i+1)*Width, (j+1)*Width, fill="white", width=1)
            temp = {}
            for action in actions:
                temp[action] = create_triangle(i, j, action)
            cell_scores[(i,j)] = temp
    for (i, j, c, w) in specials:
        board.create_rectangle(i*Width, j*Width, (i+1)*Width, (j+1)*Width, fill=c, width=1)
    for (i, j) in walls:
        board.create_rectangle(i*Width, j*Width, (i+1)*Width, (j+1)*Width, fill="black", width=1)

render_grid()


def set_cell_score(state, action, val):
    global cell_score_min, cell_score_max
    triangle = cell_scores[state][action]
    green_dec = int(min(255, max(0, (val - cell_score_min) * 255.0 / (cell_score_max - cell_score_min))))
    green = hex(green_dec)[2:]
    red = hex(255-green_dec)[2:]
    if len(red) == 1:
        red += "0"
    if len(green) == 1:
        green += "0"
    color = "#" + red + green + "00"
    board.itemconfigure(triangle, fill=color)


def try_move(dx, dy, ag_num):
    global agents, x, y, score, walk_reward, me, restart
    if restart == True:
        restart_game()
    new_x = agents[ag_num][0] + dx
    new_y = agents[ag_num][1] + dy
    score[ag_num] += walk_reward
    if (new_x >= 0) and (new_x < x) and (new_y >= 0) and (new_y < y) and not ((new_x, new_y) in walls):
        board.coords(me[ag_num], new_x*Width+Width*2/10, new_y*Width+Width*2/10, new_x*Width+Width*8/10, new_y*Width+Width*8/10)
        agents[ag_num] = (new_x, new_y)
    for (i, j, c, w) in specials:
        if new_x == i and new_y == j:
            score[ag_num] -= walk_reward
            score[ag_num] += w
            score[1-ag_num]+=w
            agent_o = agents[1-ag_num]
            if not(agent_o[0]== i and agent_o[1] == j):
                score[1-ag_num]-=np.sqrt((agent_o[0]-4)**2+(agent_o[1]-4)**2)/2
            print("Score: ", score)
            restart = True
            return
    #print "score: ", score


def call_up(event):
    try_move(0, -1)


def call_down(event):
    try_move(0, 1)


def call_left(event):
    try_move(-1, 0)


def call_right(event):
    try_move(1, 0)


def restart_game():
    global agents, score, me, restart
    agents = [(4, 0),(0,4)]
    score = [1,1]
    restart = False
    for i in range(len(agents)):
        board.coords(me[i], agents[i][0]*Width+Width*2/10, agents[i][1]*Width+Width*2/10, agents[i][0]*Width+Width*8/10, agents[i][1]*Width+Width*8/10)

def has_restarted():
    return restart

master.bind("<Up>", call_up)
master.bind("<Down>", call_down)
master.bind("<Right>", call_right)
master.bind("<Left>", call_left)

me=[0,0]

me[0] = board.create_rectangle(agents[0][0]*Width+Width*2/10, agents[0][1]*Width+Width*2/10,
                            agents[0][0]*Width+Width*8/10, agents[0][1]*Width+Width*8/10, fill="orange", width=1, tag="me0")
me[1] = board.create_rectangle(agents[1][0]*Width+Width*2/10, agents[1][1]*Width+Width*2/10,
                            agents[1][0]*Width+Width*8/10, agents[1][1]*Width+Width*8/10, fill="blue", width=1, tag="me1")

board.grid(row=0, column=0)

def start_game():
    master.mainloop()
