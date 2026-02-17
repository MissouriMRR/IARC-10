import heapq
import math

# will have to figure out how to calculate distance between nodes (weights)

#for each (safe) node in graph
# distance [node] = infinity 

'''
#lists of visited (previous) and unvisited nodes
unvisited = []
previous = []

myDict = {'a': 1, 'b': 2, 'c': 3}
removedVal = myDict.pop('b')
print(removedVal)
print(myDict)      

previous.append(removedVal)
print(previous)
'''

# creating a graph class
class Graph:
    def __init__(self, graph: dict ={}):
        self.graph = graph #a dictionary for the adjacency list
        

    def add_edge(self, node1, node2, weight):
        if node1 not in self.graph: #check if the node is already
            self.graph[node1] = {} # if not, create the node
            
        self.graph[node1][node2] = weight #else, add a connection to its neighbor
    
    def shortest_distances(self, source: str):
        #initialize the vals of all nodes with infinity
        distances = {node: float("inf") for node in self.graph}
        distances[source] = 0 #set the source value to 0

        # Initialize a priporty queue
        pq = [(0, source)]
        heapq.heapify(pq)
        
        # Create a set to hold 
        visited = set()

        # WHILE NOT EMPTY   
        #keep popping highest-priority node (min value) 
        #and extracting its value and name to currentDistance and currentNode
        #if currentNode is inside visited, skip
        #else, mark it as visited and then move on to visiting its neighbors
        while pq:
            current_distance, current_node = heapq.heappop(
                pq
            ) # get the node with the min distance


            if current_node in visited:
                continue #skip already visited nodes
            visited.add(current_node) #else, add the node to visited set

            for neighbor, weight in self.graph[current_node].items():

                #calculate the distance from currentNode to the neighbor
                tentative_distance = current_distance + weight
                if tentative_distance < distances[neighbor]:
                    distances[neighbor] = tentative_distance

                    heapq.heappush(pq, (tentative_distance, neighbor))

            predecessors = {node: None for node in self.graph}

            for node, distance in distances.items():
                for neighbor, weight in self.graph[node].items():
                    if distances[neighbor] == distance + weight:
                        predecessors[neighbor] = node

        return distances, predecessors
    #ALLOWS FOR MULTIPLE TARGETS, IS PASSED A LIST OF POSSIBLE TARGETS, returns shortest among the list
    def shortest_path(self, src: str, target_list: list[str]):
        
        #generating preds dictionary
        _, predecessors = self.shortest_distances(src)
        
        path = []
        
        #Given the list of distances, returns te lowest one
        target=target_list[0]
        for i in target_list:
            if _[i] <= _[target]:
                target=i
        
        
        current_node = target

        #back tracking from target usings preds
        while current_node:
            path.append(current_node)
            current_node = predecessors[current_node]
            
           

        # reversing path and returning it
        path.reverse()

        return path
    
    def reconstruct_path(self, came_from: dict, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)  
        total_path.reverse()
        return total_path

    def a_star(self, start: str, targets: list[str], h):
        open_set = [(0,start)]
        heapq.heapify(open_set)
        

        came_from = dict()

        g_score = dict() # default values should be INF
        g_score[start] = 0

        f_score = dict() # default values should be INF
        f_score[start] = h(start) 
    
        while open_set:
            current = heapq.heappop(open_set)[1]
            if current in targets:
                return self.reconstruct_path(came_from, current)

            for neighbor, weight in self.graph[current].items():
                tenative_gScore = g_score[current] + weight
                if neighbor not in g_score:
                    g_score[neighbor]=math.inf
                if tenative_gScore < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tenative_gScore
                    f_score[neighbor] = tenative_gScore + h(neighbor)
                    if neighbor not in open_set:
                        heapq.heappush(open_set,(f_score[neighbor],neighbor))

        return "No Path"
    
if __name__=="__main__":
    
    # dictionary set up
    # 
    graph = {
    "A": {"B": 3, "C": 3},
    "B": {"A": 3, "D": 3.5, "E": 2.8},
    "C": {"A": 3, "E": 2.8, "F": 3.5}, 
    "D": {"B": 3.5, "E": 3.1, "G": 10},
    "E": {"B": 2.8, "C": 2.8, "D": 3.1, "G": 7},
    "F": {"C": 3.5, "G": 2.5},
    "G": {"D": 10, "E": 7, "F": 2.5},
    }
    # each key of the dictionary is a node and each value is a dict 
    # containing the neighbors of the key and distance to it
    # testing implementation
    G = Graph(graph)

