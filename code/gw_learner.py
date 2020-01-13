__author__ = 'philippe'
import gworld
import threading
import time

discount = 0.3
actions = gworld.actions
states = [[],[]]
Q = [{},{}]

for k in range(2):
    for i in range(gworld.x):
        for j in range(gworld.y):    
            states[k].append((i, j))

for k in range(2):
    for state in states[k]:
        temp = {}
        for action in actions:
            temp[action] = 0.1
            gworld.set_cell_score(state, action, temp[action])
        Q[k][state] = temp

for (i, j, c, w) in gworld.specials:
    for action in actions:
        Q[0][(i, j)][action] = w
        gworld.set_cell_score((i, j), action, w)
print(states)
print(Q[0])
def do_action(action, ag_num):
    s0 = gworld.agents
    r0 = -gworld.score
    if action == actions[0]:
        gworld.try_move(0, -1, ag_num)
    elif action == actions[1]:
        gworld.try_move(0, 1, ag_num)
    elif action == actions[2]:
        gworld.try_move(-1, 0, ag_num)
    elif action == actions[3]:
        gworld.try_move(1, 0, ag_num)
    else:
        return
    s1 = gworld.agents
    r0 += gworld.score
    return s0, action, r0, s1


def max_Q(s, ag_num):
    val = None
    act = None
    for a, q in Q[ag_num][s].items():
        if (val is None) or (q > val):
            val = q
            act = a
    return act, val


def inc_Q(s, a, alpha, inc, ag_num):
    Q[ag_num][s][a] *= 1 - alpha
    Q[ag_num][s][a] += alpha * inc
    gworld.set_cell_score(s, a, Q[0][s][a])


def run():
    global discount
    time.sleep(1)
    alpha = 1
    t = 1
    while True:
        # Pick the right action
        s = gworld.agents
        s2= gworld.agents
        max_act=[0,0]
        max_val=[0,0]
        a=[0,0]
        r=a
        (max_act[0], max_val[0]) = max_Q(s[0],0)
        (max_act[1], max_val[1]) = max_Q(s[1],1)
        (s[0], a[0], r[0], s2[0]) = do_action(max_act[0], 0)
        (s[1], a[1], r[1], s2[1]) = do_action(max_act[1], 1)
        # Update Q
        (max_act[0], max_val[0]) = max_Q(s2[0],0)
        (max_act[1], max_val[1]) = max_Q(s2[1],1)
        inc_Q(s[0], a[0], alpha, r[0] + discount * max_val[0])
        inc_Q(s[1], a[1], alpha, r[1] + discount * max_val[1])
        # Check if the game has restarted
        t += 1.0
        if gworld.has_restarted():
            gworld.restart_game()
            time.sleep(0.01)
            t = 1.0

        # Update the learning rate
        alpha = pow(t, -0.1)

        # MODIFY THIS SLEEP IF THE GAME IS GOING TOO FAST.
        time.sleep(0.1)


t = threading.Thread(target=run)
t.daemon = True
t.start()
gworld.start_game()
