
# Generate maze-like grid worlds of size 101*101 with depth-first search algorithm
# using random tie breaking.

import random
import numpy
import Tkinter


# get grid size as (col, row)
def get_grid_size(matrix):

    row = len(matrix)
    col = len(matrix[0])
    size = (col, row)

    return size
# with 50% probability mark a cell as slow_cell

def set_slow_cell(list, maze_grid):
    for pt in list:
        for i in range(pt[0]-15, pt[0]+15):
            for j in range(pt[1] - 15, pt[1] + 15):
                if maze_grid[i][j]==1:
                    maze_grid[i][j] += random.randint(0, 1)
    return maze_grid

def orientation_decider():
    a = random.randint(1, 100)
    if a <= 60:
        deice = 0
    elif a <=80:
        deice= -1
    else:
        deice = 1
    return deice


def set_high_way(maze_grid):
    max_row=len(maze_grid)
    max_col=len(maze_grid[0])
    counter=0
    path_list=[]

    while counter<4:
        node_list=[]
        i,j=0,0
        length=0
        flag_inner = False
        flag_outer = False
        k=random.randint(0, 3)
        if k==0:
            j=random.randint(1, max_row-2)
        elif k==2:
            i=max_col-1
            j = random.randint(1, max_row - 2)
        elif k==3:
            i=random.randint(1, max_col - 2)
            j=max_row-1
        else:
            i = random.randint(1, max_col - 2)
        node_list.append((i,j))
        while True:
            if k % 4 == 0:
                if i+20 <max_col-1:
                    former_i=i
                    former_j=j
                    i+=20
                    #confilct detect
                    flag_inner = test_inner(former_i,former_j,i,j,node_list)
                    flag_outer = test_outer(former_i,former_j,i,j,path_list)
                    if flag_inner is True or flag_outer is True:
                        break;



                    node_list.append((i, j))
                    length+=20
                    k+=orientation_decider()
                else:
                    former_i = i
                    former_j = j
                    i=max_col-1
                    # confilct detect
                    flag_inner = test_inner(former_i, former_j, i, j, node_list)
                    flag_outer = test_outer(former_i, former_j, i, j, path_list)
                    if flag_inner is True or flag_outer is True:
                        break;

                    node_list.append((max_col-1, j))
                    length=length+max_col-1-i
                    break

            elif k % 4 == 2:
                if i - 20 > 0:
                    former_i = i
                    former_j = j
                    i -= 20
                    # confilct detect
                    flag_inner = test_inner(former_i, former_j, i, j, node_list)
                    flag_outer = test_outer(former_i, former_j, i, j, path_list)
                    if flag_inner is True or flag_outer is True:
                        break;


                    node_list.append((i, j))
                    length += 20
                    k += orientation_decider()
                else:
                    former_i = i
                    former_j = j
                    i=0
                    # confilct detect
                    flag_inner = test_inner(former_i, former_j, i, j, node_list)
                    flag_outer = test_outer(former_i, former_j, i, j, path_list)
                    if flag_inner is True or flag_outer is True:
                        break;



                    node_list.append((0, j))
                    length = length+i
                    break
            elif k % 4 == 3:
                if j - 20 > 0:
                    former_i = i
                    former_j = j
                    j -= 20
                    # confilct detect
                    flag_inner = test_inner(former_i, former_j, i, j, node_list)
                    flag_outer = test_outer(former_i, former_j, i, j, path_list)
                    if flag_inner is True or flag_outer is True:
                        break;


                    node_list.append((i, j))
                    length += 20
                    k += orientation_decider()
                else:
                    former_i = i
                    former_j = j
                    j=0
                    # confilct detect
                    flag_inner = test_inner(former_i, former_j, i, j, node_list)
                    flag_outer = test_outer(former_i, former_j, i, j, path_list)
                    if flag_inner is True or flag_outer is True:
                        break;


                    node_list.append((i, 0))
                    length=length+j
                    break
            else:
                if j + 20 < max_row-1:
                    former_i = i
                    former_j = j
                    j += 20
                    # confilct detect
                    flag_inner = test_inner(former_i, former_j, i, j, node_list)
                    flag_outer = test_outer(former_i, former_j, i, j, path_list)
                    if flag_inner is True or flag_outer is True:
                        break;



                    node_list.append((i, j))
                    length += 20
                    k += orientation_decider()
                else:
                    former_i = i
                    former_j = j
                    j=max_row-1
                    # confilct detect
                    flag_inner = test_inner(former_i, former_j, i, j, node_list)
                    flag_outer = test_outer(former_i, former_j, i, j, path_list)
                    if flag_inner is True or flag_outer is True:
                        break;



                    node_list.append((i, max_row-1))
                    length=length+max_row-1-j
                    break

        if flag_inner is False and flag_outer is False and length>99:
                path_list.append(node_list)
                counter+=1
    num_of_path=1
    for item in path_list: #notice i means row,j coloum
        length=len(item)-1
        while length>0:
            if  item[length][0]-item[length-1][0] !=0: #horizon movement
                if item[length][0]<=item[length-1][0]:
                    for i in range(item[length][0], item[length - 1][0] + 1):
                        if maze_grid[item[length][1]][i] == 1:
                            maze_grid[item[length][1]][i] = 'a'+str(num_of_path)
                        if maze_grid[item[length][1]][i] == 2:
                            maze_grid[item[length][1]][i] = 'b'+str(num_of_path)
                else:
                    for i in range(item[length-1][0], item[length][0] + 1):
                        if maze_grid[item[length][1]][i] == 1:
                            maze_grid[item[length][1]][i] = 'a'+str(num_of_path)
                        if maze_grid[item[length][1]][i] == 2:
                            maze_grid[item[length][1]][i] = 'b'+str(num_of_path)

            if item[length][1] - item[length - 1][1] != 0:
                if item[length][1]<item[length - 1][1]:
                    for i in range(item[length][1], item[length - 1][1] + 1):
                        if maze_grid[i][item[length][0]] == 1:
                            maze_grid[i][item[length][0]] = 'a'+str(num_of_path)
                        if maze_grid[i][item[length][0]] == 2:
                            maze_grid[i][item[length][0]] = 'b'+str(num_of_path)
                else:
                    for i in range(item[length-1][1], item[length][1] + 1):
                        if maze_grid[i][item[length][0]] == 1:
                            maze_grid[i][item[length][0]] = 'a'+str(num_of_path)
                        if maze_grid[i][item[length][0]] == 2:
                            maze_grid[i][item[length][0]] = 'b'+str(num_of_path)

            length-=1
        num_of_path+=1

def test_outer(former_i,former_j,i,j,path_list):
    avg_i=(former_i+i)/2
    avg_j=(former_j+j)/2
    leng_i=abs(former_i-i)
    leng_j=abs(former_j-j)
    for item in path_list:
        index = len(item)
        index -= 1
        while index > 0:
            item_avg_y = (item[index][1] + item[index - 1][1]) / 2
            item_avg_x = (item[index][0] + item[index - 1][0]) / 2
            x_dis=abs(item[index][0] - item[index - 1][0])/2+leng_i/2
            y_dis=abs(item[index][1] - item[index - 1][1])/2+leng_j/2
            if abs(item_avg_x - avg_i) <= x_dis and abs(item_avg_y - avg_j) <= y_dis:
                return True
            index -= 1
    return False


def test_inner(former_i,former_j,i,j,node_list):
    avg_i = (former_i + i) / 2
    avg_j = (former_j + j) / 2
    leng_i = abs(former_i - i)
    leng_j = abs(former_j - j)

    index = len(node_list)
    index -= 1
    while index > 0:
        if node_list[index - 1][1] == j and node_list[index - 1][0] == i:
            return True

        item_avg_y = (node_list[index][1] + node_list[index - 1][1]) / 2
        item_avg_x = (node_list[index][0] + node_list[index - 1][0]) / 2
        x_dis = abs(node_list[index][0] - node_list[index - 1][0]) / 2 + leng_i/2
        y_dis = abs(node_list[index][1] - node_list[index - 1][1]) / 2 + leng_j/2
        if abs(item_avg_x - avg_i) < x_dis and abs(item_avg_y - avg_j) < y_dis:
            return True
        index -= 1

    return False



#select the possible start and goal point
def select_start_goal(maze_grid):
    (col,row) = get_grid_size(maze_grid)
     # randomly select a start point
    left = random.randint(0,1)
    top = random.randint(0,1)
    if left:
        r = random.randint(0, 20-1)
    else:
        r = random.randint(row-1-19,row-1)
    if top:
        c = random.randint(0, 20-1)
    else:
        c = random.randint(col-1-20,col-1)
    cell = (r, c)
    return cell

#set 3840 block cell
def set_block(maze_grid):
    (col,row) = get_grid_size(maze_grid)
    block_count = 1
    while block_count<= 3840:
        r = random.randint(0,row-1)
        c = random.randint(0,col-1)
        if maze_grid[r][c] == 1 or maze_grid[r][c] == 2:
            maze_grid[r][c] = 0
            block_count += 1


# ----------------------------------- create maze ----------------------------------- #
# maze = create_maze_dfs(col, row): generate maze-like grid world of size 101*101
#  input: column, row
# output: maze_grid of size (col, row), 1 if maze_grid[row][col] blocked, 0 otherwise

def create_map(col=160, row=120):

    # for unblocked cells
    maze_grid = [[1 for i in range(col)] for i in range(row)]

    hard_traverse = []
    for i in range(8):
        r = random.randint(0+16, row-1-16)
        c = random.randint(0+16, col-1-16)
        cell = (r, c)
        hard_traverse.append(cell)
    set_slow_cell(hard_traverse,maze_grid)

    set_high_way(maze_grid)
    set_block(maze_grid)

    return maze_grid, hard_traverse

#  set start and goal point
def set_state(maze_grid, hard_traverse):
    #select the possible start and goal point
    start = select_start_goal(maze_grid)
    goal = select_start_goal(maze_grid)
    reselect_start_goal = True
    while reselect_start_goal:
        dis = (start[0]-goal[0])*(start[0]-goal[0]) + (start[1] - goal[1])*(start[1] - goal[1])
        if dis < 10000 or  maze_grid[start[0]][start[1]] == 0 or maze_grid[goal[0]][goal[1]]== 0:
            start = select_start_goal(maze_grid)
            goal = select_start_goal(maze_grid)
        else:
            reselect_start_goal = False

    maze_grid[start[0]][start[1]] = "S"
    maze_grid[goal[0]][goal[1]] = "G"

    return start, goal, maze_grid, hard_traverse

#write the map into txt file
def writemap(filename, start, goal, hard_traverse, maze_grid):
    file = open(filename,'w')
    file.write(str(start[0])+ ',' + str(start[1]) + '\n')
    file.write(str(goal[0])+ ',' + str(goal[1]) + '\n')
    for i in hard_traverse:
        k = ','.join([str(j) for j in i])
        file.write(k+"\n")
    (col,row) = get_grid_size(maze_grid)
    file.write(str(row) + ',' + str(col) + '\n')
    for i in maze_grid:
        k = ' '.join([str(j) for j in i])
        file.write(k+"\n")
    file.close()

#read the map from txt file
def readmap(filename):
    file = open(filename,'r')

    line = file.readline()
    start = line

    line = file.readline()
    goal = line

    hard_traverse = []
    for i in range(8):
        line = file.readline()
        line = line.strip('\n')
        hard_traverse.append(line)

    line = file.readline()
    maze_grid = []
    while line:
        line = file.readline()
        line = line.strip('\n')
        maze_grid.append(line)
    maze_grid.pop()

    goal = list(map(eval,goal.split(',')))

    start = list(map(eval,start.split(',')))
    for i in range(len(hard_traverse)):
        hard_traverse[i] = hard_traverse[i].split(',')
        hard_traverse[i] = list(map(eval,hard_traverse[i]))
    for i in range(len(maze_grid)):
        maze_grid[i] = maze_grid[i].split(' ')
        for index,item in enumerate(maze_grid[i]):
            if item == '0':
                maze_grid[i][index] = 0
            elif item == '1':
                maze_grid[i][index] = 1
            elif item == '2':
                maze_grid[i][index] = 2
    return start,goal,hard_traverse,maze_grid

# draw maze
def draw_map(maze_grid):

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
            # if start
            if maze_grid[r][c] == "S":
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="green")
            # if goal
            if maze_grid[r][c] == "G":
                canvas.create_rectangle((c+1)*5+1, (r+1)*5+1, (c+2)*5, (r+2)*5, fill="red")
    screen.mainloop()

#test
# (mymap,hard_traverse) = create_map(160,120)
# (start,goal,mymap,hard_traverse) = set_state(mymap,hard_traverse)
# writemap('test.txt',start,goal,hard_traverse,mymap)
#
# rstart,rgoal,rhard, rmap = readmap('test.txt')
#draw_map(rmap)
