import random
from simpleai.search import SearchProblem, hill_climbing, hill_climbing_random_restarts, genetic, breadth_first, astar, uniform_cost, depth_first, greedy, astar, limited_depth_first, iterative_limited_depth_first

class NQueens(SearchProblem):
    def __init__(self, N, initial_state=None):
        super().__init__(initial_state)
        self.N = N

    def actions(self, state):
        possible_actions = []
        for i in range(1, self.N + 1):
            for j in range(1, self.N + 1):
                possible_actions.append(('Move Queen {} to row {}'.format(i, j), i, j))
        return possible_actions

    def result(self, state, action):
        _, i, j = action
        new_state = state[:i - 1] + str(j) + state[i:]
        return new_state

    def is_goal(self, state):
        return self._count_attacking_pairs(state) == 0

    def heuristic(self, state):
        return self._count_attacking_pairs(state)

    def _count_attacking_pairs(self, state):
        count = 0
        for i in range(self.N):
            for j in range(i + 1, self.N):
                if state[i] == state[j] or abs(int(state[i]) - int(state[j])) == abs(i - j):
                    count += 1
        return count

    def value(self, state):
        return (self.N * (self.N - 1)) // 2 - self._count_attacking_pairs(state)

    def generate_random_state(self):
        return ''.join(str(random.randint(1, self.N)) for _ in range(self.N))

    def crossover(self, state1, state2):
        crossover_point = random.randint(1, self.N - 1)
        return state1[:crossover_point] + state2[crossover_point:]

    def mutate(self, state):
        index_to_mutate = random.randint(0, self.N - 1)
        new_value = random.randint(1, self.N)
        return state[:index_to_mutate] + str(new_value) + state[index_to_mutate + 1:]

def test_algorithms(N, initial_states):
    for initial_state in initial_states:
        problem = NQueens(N, initial_state)
        algorithms = [
            breadth_first,
            astar,
            hill_climbing,
            hill_climbing_random_restarts,
            genetic,
            uniform_cost,
            # depth_first,
            limited_depth_first,
            iterative_limited_depth_first,
            greedy,
        ]

        for algorithm in algorithms:
            print('*' * 50)
            print(f'{algorithm.__name__}')

            if algorithm == genetic:
                result = algorithm(problem, population_size=20,
                                   mutation_chance=0.2)
            elif algorithm == hill_climbing:
                result = hill_climbing(problem, iterations_limit=1000)
            elif algorithm == limited_depth_first:
                result = algorithm(problem, depth_limit=5)
            elif algorithm == hill_climbing_random_restarts:
                result = algorithm(problem,restarts_limit=5)
            else:
                result = algorithm(problem)

            print('Resulting path:')
            print(result.path())
            print('Resulting state:', result.state)
            print('Total costs:', result.cost)
            print('Value:', problem.value(result.state))
            print('Correct solution?:', problem.is_goal(result.state))
            print('*' * 50)

test_algorithms(4, ['2323', '4311', '3442', '1234'])
# test_algorithms(7, ['2323232', '4311431', '3442342', '1234512'])
