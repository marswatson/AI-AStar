from CreateMap import create_map, set_state, draw_map, writemap, readmap
from A_star import create_map, A_star, draw_path,write_path
import copy
import thread
from time import clock

if __name__=="__main__":
    choose  =  '4'
    while choose != '3':
        print "Please input your choice:"
        print "1.Create new map"
        print "2.Read map from file"
        print "3.Exit"
        choose = raw_input("Enter your choice:_")
        if choose == '1':
            print "Please input the filename"
            name = raw_input("Input file name:_")
            (my_map,hard_traverse) = create_map(160, 120)
            (start, goal, map, hard_traverse) = set_state(my_map, hard_traverse)
            writemap(name+'.txt',start,goal,hard_traverse,map)
            print "new map have been created."
            h_style = raw_input("Input heuristic style('d','m','e','p','o'):_")
            weight = raw_input("Input weight value:_")
            draw_map(my_map)
            t0 = clock()
            (path, reach_goal, expanded, states) = A_star(start, goal, my_map, float(weight), h_style, "large")
            t1 = clock()
            time=t1-t0

            #another thread to draw map
            thread.start_new_thread( draw_path, (my_map,path) )

            print path
            print "run time is: " + str(time) + ". Expanded nodes: " + str(expanded)

            view = 'y'
            while view == 'y':
                print "please input the row and col to see cost value"
                r = raw_input("row:_")
                c = raw_input("col:_")
                r = int(r)
                c = int(c)
                if int(r) <= 120 and int(r)>=1 and int(c) <= 160 and int(c)>=1:
                    print "f = " + str(states[r-1][c-1].f) + ", g = " + str(states[r-1][c-1].g) + ", h = " + str(states[r-1][c-1].h)
                view = raw_input("do you want to view another cell(y/n): ")

            (r,c) = path[-1]
            totalcost = states[r][c].g
            filename = name[0:len(name)-4] + "_result.txt"
            write_path(filename, totalcost, path)
        elif choose == '2':
            name = raw_input("Input file name:_")
            (start, goal, hard_traverse, my_map)= readmap(name)
            h_style = raw_input("Input heuristic style('d','m','e','p','o'):_")
            weight = raw_input("Input weight value:_")

            draw_map(my_map)
            print "success read map"

            t0 = clock()
            (path, reach_goal, expanded, states) = A_star(start, goal, my_map, float(weight), h_style, "large")
            t1 = clock()
            time=t1-t0

            #another thread to draw map
            thread.start_new_thread( draw_path, (my_map,path) )

            print path
            print "run time is: " + str(time) + ". Expanded nodes: " + str(expanded)

            view = 'y'
            while view == 'y':
                print "please input the row and col to see cost value"
                r = raw_input("row:_")
                c = raw_input("col:_")
                r = int(r)
                c = int(c)
                if int(r) <= 120 and int(r)>=1 and int(c) <= 160 and int(c)>=1:
                    print "f = " + str(states[r-1][c-1].f) + ", g = " + str(states[r-1][c-1].g) + ", h = " + str(states[r-1][c-1].h)
                view = raw_input("do you want to view another cell(y/n): ")

            (r,c) = path[-1]
            totalcost = states[r][c].g
            filename = name[0:len(name)-4] + "_result.txt"
            write_path(filename, totalcost, path)

        elif choose == '3':
            print "Exit"
        else:
            continue

