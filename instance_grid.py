from collections import defaultdict
import random as rand


def generate_network(n):
    edges = []
    for i in range(n):
        for j in range(n):
            if j != n-1:
                edges.append([j+i*n, j+i*n+1])
            if i != n-1:
                edges.append([j+i*n, j+(i+1)*n])
    graph = defaultdict(list)
    for edge in edges:
        a, b = edge[0], edge[1]
         
        graph[a].append(b)
        graph[b].append(a)
    return graph

def set_speed(graph, n, v):
    V = [[0 for j in range(n*n)]for i in range(n*n)]
    for ind in graph.keys():
        for neigh in graph[ind]:
            V[ind][neigh] = v
    return V

def set_distance(graph, n, d):
    D = [[0 for j in range(n*n)]for i in range(n*n)]
    for ind in graph.keys():
        for neigh in graph[ind]:
            D[ind][neigh] = d
    return D

def shortest_path(graph, start ,goal):
	explored = []
	
	queue = [[start]]
	
	if start == goal:
		print("Same Node")
		return
	while queue:
		path = queue.pop(0)
		node = path[-1]
		
		if node not in explored:
			neighbours = graph[node]
			
			for neighbour in neighbours:
				new_path = list(path)
				new_path.append(neighbour)
				queue.append(new_path)
				
				if neighbour == goal:
					return new_path
			explored.append(node)
	return

def generate_flights(k, n, graph, seed):
    F = []
    rand.seed(seed)
    for i in range(k):
        s = rand.randint(0,n*n-1)
        e = s
        while e==s:
            e = rand.randint(0,n*n-1)
        F.append(shortest_path(graph, s, e))
    #print(F)
    return F

def get_conflicts(F, D, v,n): #very specific to grid and metroplex
    conf = []
    tau = D/v 
    for i in range(len(F)):
        for j in range(i):
            Fi, Fj = set(F[i]), set(F[j])
            tmp = Fi & Fj
            for c in tmp:
                conf.append([i,j,c, tau])
    return conf
def matrix_to_string(data):
    tmp = ""
    for i in range(len(data)):
        for x in data[i]:
            tmp += str(x) + " "
        tmp += '\n'
    return tmp

def matrix_to_string_ID(data):
    tmp = ""
    for i in range(len(data)):
        tmp += str(i) + " "
        for x in data[i]:
            tmp += str(x) + " "
        tmp += '\n'
    return tmp

def inst_to_file(seed):

    n = 5
    D = 2
    d = 10
    k = 10
    v = 1
    
    filename = "testgrid_"+str(seed)+".tsruami"
    
    G = generate_network(n)
    #print('graph', G)
    #print(shortest_path(G, 0, 3))
    F = generate_flights(k, n, G, seed)
    Dist = set_distance(G,n,d)
    V = set_speed(G,n,v)
    conf = get_conflicts(F,D,v,n)
    #print('conf', conf)

    buff = ""
    buff += str(len(G)) + " " + str(k) + " "+ str(len(conf))+"\n"

    buff += matrix_to_string(Dist)
    buff += matrix_to_string(V)

    buff += matrix_to_string_ID(F)
    buff += matrix_to_string_ID(conf)

    file = open(filename, 'w')
    file.write(buff)
    file.close()
    
    

seed = 1
for seed in range(0,1):
    inst_to_file(seed)

