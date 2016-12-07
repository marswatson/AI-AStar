
# The A* algorithm is for shortest path from start to goal in a grid maze
# The algorithm has many different choices:
# choose large or small g when f values are equal; forward or backward; adaptive or not

#from memory_profiler import profile
from CreateMap import create_map, set_state, draw_map, get_grid_size, writemap, readmap
from Classes import Heap, State
import Tkinter

# ------------------------------ initialization ------------------------------ #

# create original grid of state of size (col, row)
def create_state(row=120, col=160, weight = 1):

    # create grid
    state_grid = [[State() for i in range(col)] for i in range(row)]

    # change values in state
    for i in range(col):
        for j in range(row):
            # correct position
            state_grid[j][i].position = (j, i)
            state_grid[j][i].position = (j, i)
            state_grid[j][i].w = weight
            # correct movement
            # if the first column, remove all left
            if i == 0:
                state_grid[j][i].actions[0] = 0
                state_grid[j][i].actions[3] = 0
                state_grid[j][i].actions[5] = 0
            # if the first row, remove all up
            if j == 0:
                state_grid[j][i].actions[0] = 0
                state_grid[j][i].actions[1] = 0
                state_grid[j][i].actions[2] = 0
            # if the last column, remove all right
            if i == col-1:
                state_grid[j][i].actions[2] = 0
                state_grid[j][i].actions[4] = 0
                state_grid[j][i].actions[6] = 0
            # if the last row, remove all down
            if j == col-1:
                state_grid[j][i].actions[5] = 0
                state_grid[j][i].actions[6] = 0
                state_grid[j][i].actions[7] = 0

    return state_grid


# ------------------------------- start A* ------------------------------------ #

# global movement first move up and down,,second move left and right
move = [[-1, -1, -1, 0, 0, 1, 1, 1], [-1, 0, 1, -1, 1, -1, 0, 1]]


# action is int from 0 to 3, represents left, right, up, down; position is (c, r); state is the state grid
def get_succ(states, position, action):

    # movement [[0, 0, up, down], [left, right, 0, 0]]
    global move

    (r, c) = position
    # get next position
    r_new = r + move[0][action]
    c_new = c + move[1][action]
    succ = states[r_new][c_new]

    return succ

# --------------------------------------------- #
# search for a new path when original one blocked
def compute_path(maze_grid, states, goal, open_list, close_list, counter, expanded,heuristic_style, large_g):

    global move

    (rg, cg) = goal

    while states[rg][cg].g > open_list.heap[-1][1].f:

        # open_list is binary heap while each item contains two value: f, state
        # remove the smallest f value item from open_list heap
        last_item = open_list.pop()
        # add the state with smallest f value into close_list
        close_list.append(last_item[1].position)
        #print close list
        #print str(len(close_list)) + '\n'

        (r, c) = last_item[1].position

        for i in range(8):

            if states[r][c].action_exist(i,maze_grid):
                # get the successor of last item when action is i
                pos = (r, c)
                successor = get_succ(states, pos, i)

                # the position of successor
                r_succ = r + move[0][i]
                c_succ = c + move[1][i]

                try:
                    close_list.index((r_succ, c_succ))
                except ValueError:

                    if successor.search < counter:

                        # the successor state in state_grid
                        states[r_succ][c_succ].g = 10000
                        states[r_succ][c_succ].renew_hf(heuristic_style,goal)
                        states[r_succ][c_succ].search = counter

                    if successor.g > states[r][c].g + states[r][c].cost(i, maze_grid):
                        states[r_succ][c_succ].g = states[r][c].g + states[r][c].cost(i,maze_grid)
                        states[r_succ][c_succ].renew_hf(heuristic_style,goal)
                        states[r_succ][c_succ].tree = states[r][c]

                        succ_state = states[r_succ][c_succ]
                        # choose favor of large g or small g when f is equal
                        if large_g == "large":
                            succ_favor = states[r_succ][c_succ].f * 10000 - states[r_succ][c_succ].g
                        else:
                            succ_favor = states[r_succ][c_succ].f * 10000 + states[r_succ][c_succ].g

                        j = 0
                        while j < len(open_list.heap) and len(open_list.heap) > 0:

                            # if successor is in open list
                            if succ_state.position == open_list.heap[j][1].position:

                                # renew the g value in the corresponding open list item and renew heap
                                open_list.remove(j)

                            j += 1

                        # insert the successor into open list
                        open_list.insert([succ_favor, succ_state])
                        expanded += 1
                else:
                    continue

        if len(open_list.heap) == 0:
            break

    return states, open_list, close_list, expanded


# --------------------------------------- A* search ---------------------------------------- #

def A_star(start, goal, maze_grid, weight, heuristic_style='d', large_g="large"):

    # initial state grid
    states = create_state(len(maze_grid), len(maze_grid[0]), weight)

    expanded = 0
    counter = 1
    current = start
    (rg, cg) = goal
    path = []
    path2 = []

    (rs, cs) = current
    states[rs][cs].g = 0
    states[rs][cs].renew_hf(heuristic_style,goal)
    states[rs][cs].search = counter
    states[rg][cg].g = 10000
    states[rg][cg].renew_hf(heuristic_style,goal)
    states[rg][cg].search = counter

    # open_list is binary heap while each item contains two value: f, state
    open_list = Heap()
    # close_list only store state
    close_list = []

    open_state = states[rs][cs]
    # choose favor of large g or small g when f is equal
    if large_g == "large":
        open_favor = states[rs][cs].f * 1000 - states[rs][cs].g
    else:
        open_favor = states[rs][cs].f * 1000 + states[rs][cs].g

    # insert s_start into open list
    open_list.insert([open_favor, open_state])
    # compute path
    (states, open_list, close_list, expanded) = compute_path(maze_grid, states, goal, open_list, close_list, counter, expanded, heuristic_style,large_g)
    if len(open_list.heap) == 0:
        print ("I cannot reach the target.")
        reach_goal = False
        return path, reach_goal, expanded, states

    # get the path from goal to start by tree-pointer
    (rc, cc) = goal
    path = [(rc, cc)]

    while (rc, cc) != (rs, cs):
        (rc, cc) = states[rc][cc].tree.position
        path.append((rc, cc))

    path.reverse()
    reach_goal = True

    return path, reach_goal, expanded, states


# draw the path from start to goal
def draw_path(maze_grid, path):

    # get size
    (col, row) = get_grid_size(maze_grid)

    screen = Tkinter.Tk()
    canvas = Tkinter.Canvas(screen, width=(col+2)*5, height=(row+2)*5)
    canvas.pack()

    # create initial grid world
    for c in range(1, col+2):
        canvas.create_line(c*5, 5, c*5, (row+1)*5, width=1)
    for r in range(1, row+2):
        canvas.create_line(5, r*5, (col+1)*5, r*5+1, width=1)

    # mark blocked grid as black, start state as red, goal as green
    for c in range(0, col):
        for r in range(0, row):

            # if blocked
            if maze_grid[r][c] == 0:
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="black")
            # if unblocked
            if maze_grid[r][c] == 1:
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="white")
            # if hard unblock
            if maze_grid[r][c] == 2:
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="gray")
            # if unblock highway
            if maze_grid[r][c] == 'a1' or maze_grid[r][c] == 'a2' or maze_grid[r][c] == 'a3' or maze_grid[r][c] == 'a4':
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="yellow")
            # if hard unblock highway
            if maze_grid[r][c] == 'b1' or maze_grid[r][c] == 'b2' or maze_grid[r][c] == 'b3' or maze_grid[r][c] == 'b4':
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="orange")
            # if the path
            if (r, c) in path:
                # mark path as blue
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="blue")
            # if start
            if maze_grid[r][c] == "S":
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="green")
            # if goal
            if maze_grid[r][c] == "G":
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="red")

    screen.mainloop()

def write_path(filename, cost, path):
    file = open(filename,'w')
    file.write(str(cost) + '\n')
    for item in path:
        file.write(str(item[0] + 1)+ ',' + str(item[1] + 1) + '\n')
    file.close()

