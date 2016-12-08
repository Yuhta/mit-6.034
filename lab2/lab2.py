# Fall 2012 6.034 Lab 2: Search
#
# Your answers for the true and false questions will be in the following form.
# Your answers will look like one of the two below:
#ANSWER1 = True
#ANSWER1 = False

# 1: True or false - Hill Climbing search is guaranteed to find a solution
#    if there is a solution
ANSWER1 = False

# 2: True or false - Best-first search will give an optimal search result
#    (shortest path length).
#    (If you don't know what we mean by best-first search, refer to
#     http://courses.csail.mit.edu/6.034f/ai3/ch4.pdf (page 13 of the pdf).)
ANSWER2 = False

# 3: True or false - Best-first search and hill climbing make use of
#    heuristic values of nodes.
ANSWER3 = True

# 4: True or false - A* uses an extended-nodes set.
ANSWER4 = True

# 5: True or false - Breadth first search is guaranteed to return a path
#    with the shortest number of nodes.
ANSWER5 = True

# 6: True or false - The regular branch and bound uses heuristic values
#    to speed up the search for an optimal path.
ANSWER6 = False

# Import the Graph data structure from 'search.py'
# Refer to search.py for documentation
import abc
import heapq

from future.utils import with_metaclass

class Agenda(with_metaclass(abc.ABCMeta)):
    @abc.abstractmethod
    def enqueue(self, path):
        raise NotImplementedError

    @abc.abstractmethod
    def dequeue(self):
        raise NotImplementedError

    @abc.abstractmethod
    def __bool__(self):
        raise NotImplementedError

    def __nonzero__(self):
        return self.__bool__()

class FILOAgenda(Agenda):
    def __init__(self):
        from collections import deque
        self.queue = deque()

    def enqueue(self, path):
        self.queue.append(path)

    def dequeue(self):
        return self.queue.pop()

    def __bool__(self):
        return bool(self.queue)

class FIFOAgenda(FILOAgenda):
    def dequeue(self):
        return self.queue.popleft()

class BeamSearchAgenda(FIFOAgenda):
    def __init__(self, width, estimate):
        super(BeamSearchAgenda, self).__init__()
        self.width = width
        self.estimate = estimate
        self.best = []

    def dequeue(self):
        if not self.best:
            d = len(self.queue[0])
            while self.queue and len(self.queue[0]) == d:
                self.best.append(self.queue.popleft())
            self.best = heapq.nsmallest(self.width, self.best, key=self.estimate)
            self.best.reverse()
        return self.best.pop()

    def __bool__(self):
        return super(BeamSearchAgenda, self).__bool__() or bool(self.best)

class SortedAgenda(Agenda):
    def __init__(self, estimate):
        self.queue = []
        self.estimate = estimate

    def __bool__(self):
        return bool(self.queue)

    def enqueue(self, path):
        heapq.heappush(self.queue, (self.estimate(path), path))

    def dequeue(self):
        return heapq.heappop(self.queue)[1]

class Search:
    def __init__(self, create_agenda, track_extended):
        self.create_agenda = create_agenda
        self.track_extended = track_extended

    def sort_paths(self, paths):
        pass

    def __call__(self, graph, start, goal):
        from pyrsistent import l
        agenda = self.create_agenda()
        assert isinstance(agenda, Agenda)
        agenda.enqueue(l(start))
        extended = set([start])
        while agenda:
            path = agenda.dequeue()
            if path.first == goal: return list(reversed(path))
            paths = [path.cons(n) for n in graph.get_connected_nodes(path.first)
                     if not (self.track_extended and n in extended or n in path)]
            self.sort_paths(paths)
            for p in paths:
                agenda.enqueue(p)
                extended.add(p.first)
        return []

## Optional Warm-up: BFS and DFS
# If you implement these, the offline tester will test them.
# If you don't, it won't.
# The online tester will not test them.

def bfs(graph, start, goal):
    search = Search(FILOAgenda, True)
    return search(graph, start, goal)

## Once you have completed the breadth-first search,
## this part should be very simple to complete.
def dfs(graph, start, goal):
    search = Search(FIFOAgenda, True)
    return search(graph, start, goal)


## Now we're going to add some heuristics into the search.
## Remember that hill-climbing is a modified version of depth-first search.
## Search direction should be towards lower heuristic values to the goal.
def hill_climbing(graph, start, goal):
    search = Search(FILOAgenda, False)
    def sort_paths(paths):
        paths.sort(key=lambda p: graph.get_heuristic(p.first, goal),
                   reverse=True)
    search.sort_paths = sort_paths
    return search(graph, start, goal)

## Now we're going to implement beam search, a variation on BFS
## that caps the amount of memory used to store paths.  Remember,
## we maintain only k candidate paths of length n in our agenda at any time.
## The k top candidates are to be determined using the
## graph get_heuristic function, with lower values being better values.
def beam_search(graph, start, goal, beam_width):
    def estimate(path): return graph.get_heuristic(path.first, goal)
    search = Search(lambda: BeamSearchAgenda(beam_width, estimate), False)
    return search(graph, start, goal)

## Now we're going to try optimal search.  The previous searches haven't
## used edge distances in the calculation.

## This function takes in a graph and a list of node names, and returns
## the sum of edge lengths along the path -- the total distance in the path.
def path_length(graph, node_names):
    l = 0
    n0 = node_names[0]
    for n in node_names:
        if n != n0: l += graph.get_edge(n0, n).length
        n0 = n
    return l


def branch_and_bound(graph, start, goal):
    search = Search(lambda: SortedAgenda(lambda p: path_length(graph, p)), False)
    return search(graph, start, goal)

def a_star(graph, start, goal):
    def estimate(p):
        return path_length(graph, p) + graph.get_heuristic(p.first, goal)
    search = Search(lambda: SortedAgenda(estimate), True)
    return search(graph, start, goal)


## It's useful to determine if a graph has a consistent and admissible
## heuristic.  You've seen graphs with heuristics that are
## admissible, but not consistent.  Have you seen any graphs that are
## consistent, but not admissible?

from future.utils import viewitems

def distances(graph, goal):
    todo = [goal]
    dist = {goal: 0}
    while todo:
        n0 = todo.pop()
        for n in graph.get_connected_nodes(n0):
            d = dist[n0] + graph.get_edge(n, n0).length
            if n in dist:
                if d < dist[n]: dist[n] = d
            else:
                dist[n] = d
                todo.append(n)
    return dist

def is_admissible(graph, goal):
    for (n, d) in viewitems(distances(graph, goal)):
        if graph.get_heuristic(n, goal) > d:
            return False
    return True

def is_consistent(graph, goal):
    for e in graph.edges:
        if abs(graph.get_heuristic(e.node1, goal) -
               graph.get_heuristic(e.node2, goal)) > e.length:
            return False
    return True

HOW_MANY_HOURS_THIS_PSET_TOOK = '42'
WHAT_I_FOUND_INTERESTING = '42'
WHAT_I_FOUND_BORING = '42'
