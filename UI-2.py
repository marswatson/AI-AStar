from CreateMap import create_map, set_state, draw_map, writemap, readmap
from A_star import create_map, A_star, draw_path,write_path
from A_star_Integrated import create_map, A_star_Integrated, draw_path,write_path
from A_star_sequential import create_map, A_star_sequential, draw_path,write_path
import copy
import thread
from time import clock


def three_A_star(start1, goal1, my_map1):
    choice = '4'
    while choice != '3':
        print("Please input your choice:")
        print("1.Run A*")
        print("2.Run sequence A*")
        print("3.Run integrated A*")
        choice = raw_input("Enter your choice:_")
        if choice == '1':
            h_style = raw_input("Input heuristic style('d','m','e','p','o'):_")
            weight = input("Input weight value:_")

            t0 = clock()
            (path, reach_goal, expanded, states) = A_star(start1, goal1, my_map1, float(weight), h_style, "large")
            t1 = clock()
            time = t1 - t0
            (r, c) = path[-1]
            totalcost = states[r][c].g
            return path, expanded, states, totalcost, time
        elif choice == '2':
            weight1 = input("Input weight 1:_")
            weight2 = input("Input weight 2:_")
            t0 = clock()
            (path, expanded, states, totalcost) = A_star_sequential(start1, goal1, my_map1, float(weight1), float(weight2))
            t1 = clock()
            time = t1 - t0
            return path, expanded, states, totalcost, time
        elif choice == '3':
            weight1 = input("Input weight 1:_")
            weight2 = input("Input weight 2:_")
            t0 = clock()
            (path, expanded, states) = A_star_Integrated(start1, goal1, my_map1, float(weight1), float(weight2))
            t1 = clock()
            time = t1 - t0
            (r, c) = path[-1]
            totalcost = states[r][c].g
            return path, expanded, states, totalcost, time
        else:
            continue


if __name__=="__main__":
    choose  =  '4'
    while choose != '3':
        print ("Please input your choice:")
        print ("1.Create new map")
        print ("2.Read map from file")
        print ("3.Exit")
        choose = raw_input("Enter your choice:_")
        if choose == '1':
            print ("Please input the filename")
            name = raw_input("Input file name:_")
            (my_map,hard_traverse) = create_map(160, 120)
            (start, goal, map, hard_traverse) = set_state(my_map, hard_traverse)
            writemap(name,start,goal,hard_traverse,map)
            print ("new map have been created.")
            draw_map(my_map)

            (path, expanded, states, totalcost, time)=three_A_star(start, goal, my_map)

            #another thread to draw map
            thread.start_new_thread( draw_path, (my_map,path) )
            print (path)
            print ("run time is: " + str(time) + ". Expanded nodes: " + str(expanded))

            view = 'y'
            while view == 'y':
                print ("please input the row and col to see cost value")
                r = input("row:_")
                c = input("col:_")
                r = int(r)
                c = int(c)
                if int(r) <= 120 and int(r)>=1 and int(c) <= 160 and int(c)>=1:
                    if len(states) > 100:
                        print("f = " + str(states[r - 1][c - 1].f) + ", g = " + str(
                            states[r - 1][c - 1].g) + ", h = " + str(states[r - 1][c - 1].h))
                    else:
                        for index in range(len(states)):
                            print("f = " + str(states[index][r - 1][c - 1].f) + ", g = " + str(
                                states[index][r - 1][c - 1].g) + ", h = " + str(states[index][r - 1][c - 1].h))

                view = raw_input("do you want to view another cell(y/n): ")

            (r,c) = path[-1]
            filename = name[0:len(name)-4] + "_result.txt"
            write_path(filename, totalcost, path)
        elif choose == '2':
            name = raw_input("Input file name:_")
            (start, goal, hard_traverse, my_map)= readmap(name)
            draw_map(my_map)
            print ("success read map")

            (path, expanded, states, totalcost,time) = three_A_star(start, goal, my_map)

            #another thread to draw map
            thread.start_new_thread( draw_path, (my_map,path) )

            print (path)
            print (("run time is: " + str(time) + ". Expanded nodes: " + str(expanded)))

            yy = 'y'
            while yy == 'y':
                print ("please input the row and col to see cost value")
                r = input("row:_")
                c = input("col:_")
                r = int(r)
                c = int(c)
                if int(r) <= 120 and int(r)>=1 and int(c) <= 160 and int(c)>=1:
                    if len(states)>100:
                        print("f = " + str(states[r - 1][c - 1].f) + ", g = " + str(
                            states[r - 1][c - 1].g) + ", h = " + str(states[r - 1][c - 1].h))
                    else:
                        for index in range(len(states)):
                            print("f = " + str(states[index][r - 1][c - 1].f) + ", g = " + str(
                                states[index][r - 1][c - 1].g) + ", h = " + str(states[index][r - 1][c - 1].h))


                yy = raw_input("do you want to view another cell(y/n): ")

            (r,c) = path[-1]
            filename = name[0:len(name)-4] + "_result.txt"
            write_path(filename, totalcost, path)

        elif choose == '3':
            print ("Exit")
        else:
            continue
