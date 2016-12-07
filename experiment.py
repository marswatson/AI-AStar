from CreateMap import create_map, set_state, draw_map, writemap, readmap
from A_star import create_map, A_star, draw_path,write_path
import copy
from time import clock
#import xlwt
#from memory_profiler import memory_usage
# 50 start points
starts = []
# 50 goals
goals = []
# 50 goals
hard_traverse_s = []
# 50 grid mazes (with starts and goals)
maps = []

# create 5 different maps, each map have 10 different start and goal
# for i in range(5):
#     (my_map,hard_traverse) = create_map(160, 120)
#     for j in range(10):
#         temp_map = copy.deepcopy(my_map)
#         (start, goal, map, hard_traverse) = set_state(temp_map, hard_traverse)
#         starts.append(start)
#         goals.append(goal)
#         hard_traverse_s.append(hard_traverse)
#         maps.append(map)
#         writemap('maps/map_' + str(i) + '_' + str(j) + '.txt',starts[i*10+j], goals[i*10+j],  hard_traverse, maps[i*10+j])
#
# if __name__=="__main__":
#     choose  =  '6'
#     while choose != '5':
#         print "Please input your choice:\n"
#         print "Create new map\n"
#         print "Read map from file\n"
#         print "Execute Algorithm\n"
#         print "Save result in file\n"
#         print "Exit"
#         choice = raw_input("Enter your choice:_")
#         if choose == '1':
#             print "Please input the filename\n"
#             name = raw_input("Input file name:_")
#             (my_map,hard_traverse) = create_map(160, 120)
#             (start, goal, map, hard_traverse) = set_state(my_map, hard_traverse)
#             writemap('test.txt',start,goal,hard_traverse,map)
#         elif choose == '2':
#             print "Read map from file\n"
#         elif choose == '3':
#             print "Execute Algorithm\n"
#
#             algorithm_name = raw_input("Input file name:_")
#             if algorithm_name == "A*":
#         elif choose == '4':
#             print "Save result in file\n"
#         elif choose == '5':
#             print "Exit"
#         else:
#             continue

# draw maze (should close the graph to continue)

# test

# (mymap,hard_traverse) = create_map(160,120)
# (start,goal,mymap,hard_traverse) = set_state(mymap,hard_traverse)
# writemap('test.txt',start,goal,hard_traverse,mymap)
# draw_map(mymap)
# (path, reach_goal, expanded, states) = A_star(start, goal, mymap, 1, 'o', "large")
# draw_path(mymap,path)
# print path
# print expanded
# print len(path)


heuristic_style_list=['d','e','m','o','p']
weight_list=[1,1.25,2]
# heuristic_style_list=['p']
# weight_list=[1,1.25,2]

file = open("exp_res.txt",'w')

for heuristic_style in heuristic_style_list:
    for weight in weight_list:
        temp = 0
        for i in range(5):
            for j in range(10):
                (start, goal, hard_traverse, mymap) = readmap('maps/map_'+str(i)+'_'+str(j)+'.txt')
                # get path by A* search

                t0 = clock()
                '''
                mem_usage = memory_usage(( A_star,(start, goal, mymap, weight, heuristic_style, "large") ))
                print (max(mem_usage))
                '''
                (path, reach_goal, expanded, states) = A_star(start, goal, mymap, weight, heuristic_style, "large")
                t1 = clock()
                time=t1-t0
                (r, c) = path[-1]
                total_cost = states[r][c].g
                write_path("path/path_"+str(i)+'_'+str(j)+'_'+heuristic_style+'_'+str(weight)+".txt", total_cost, path)
                file.write(str(states[r][c].f ) + ','+str(total_cost ) + ',' + str(expanded) + ',1,'+str( time)+',')
                temp += expanded
                file.write( '\n')
        print weight,':',temp / 1


file.close()





'''
for item in path:
    (r,c) = item
    print 'f: '+ str(states[r][c].f) + ',h: ' + str(states[r][c].h) +  ',g:'+  str(states[r][c].g) + '\n'
# if path exist, draw maze and path
if reach_goal:
    print len(path)
    print expanded
    draw_path(mymap, path)
else:
    print "Path doesn't exist"
'''



