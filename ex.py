from pddl.pddl_parser import PDDLParser
import queue as queue
import sys
from pddl.pddl_planner import PDDLPlanner
from pddl.heuristic import Heuristic
from pddl.action import Action


def applicable(state, precondition):
    positive, negative = precondition
    return positive.issubset(state) and not negative.intersection(state)


def apply(state, effects):
    positive, negative = effects
    return frozenset(state.difference(negative).union(positive))


# Objects example
# An action to move robot r1 from location l1 to location l2
a1 = Action(
    'move',  # Action name
    ['r1', 'l1', 'l2'],  # Parameters
    # Positive preconditions
    frozenset({('at', 'r1', 'l1'), ('adjacent', 'l1', 'l2')}),
    frozenset({('occupied', 'l2')}),  # Negative preconditions
    frozenset({('at', 'r1', 'l2'), ('occupied', 'l2')}),  # Add effects
    frozenset({('at', 'r1', 'l1'), ('occupied', 'l1')})  # Del effects
)

# Get each element from the action
# print(a1.name)
# print(a1.parameters)
# print(a1.positive_preconditions)
# print(a1.negative_preconditions)
# print(a1.add_effects)
# print(a1.del_effects)

# print('---------------------------------------------')

# The list of actions contains all possible actions
actions = [
    a1,
    # ...
]

# Only positive literals are present in the initial state
initial_state = frozenset({
    ('on', 'ca', 'pallet'),
    ('at', 'r1', 'l1'),
    ('belong', 'k1', 'l1'),
    ('adjacent', 'l1', 'l2'), ('adjacent', 'l2', 'l1'), ('attached', 'q2', 'l2'),
    ('empty', 'k2'),
    ('attached', 'p1', 'l1'), ('occupied', 'l1'),
    ('empty', 'k1'),
    # ...
})

# Goal literals are split in two, positive and negative
positive_goal = frozenset({('in', 'cb', 'p1'), ('in', 'ca', 'p1')})
negative_goal = frozenset()

# Test if the action move (variable a1) is applicable in our initial state (initial_state)
applicable_action = applicable(
    initial_state, (a1.positive_preconditions, a1.negative_preconditions))
# print('Is the action move applicable?', applicable_action)

# print('---------------------------------------------')

# Apply the action move in the initial state
resulting_state = apply(initial_state, (a1.add_effects, a1.del_effects))
# print('Resulting state:')
# for predicate in resulting_state:
#     print(predicate)

# print('---------------------------------------------')

# Test if the goal was achieved
goal_achieved = applicable(resulting_state, (positive_goal, negative_goal))
# print('Was the goal achieved?', goal_achieved)

# print('---------------------------------------------')

# The output plan from the planner is either a list of actions or failure (None)
# An empty plan is valid
plan = []
# Preconditions and effects are empty when obtained from a plan file, may be filled when obtained from the planner
plan = [
    Action('take', ['k1', 'cc', 'cb', 'p1', 'l1'], [], [], [], []),
    Action('load', ['k1', 'r1', 'cc', 'l1'], [], [], [], []),
    Action('move', ['r1', 'l1', 'l2'], [], [], [], []),
    Action('unload', ['k2', 'r1', 'cc', 'l2'], [], [], [], [])
    # ...
]
# Failure
plan = None

# A valid plan is either true or false
valid_plan = True
invalid_plan = False


class MaxHeuristic(Heuristic):
    # Positive goals and negative goals in a tuple
    def h(self, actions, state, goals):
        i = 0
        factlevel = [state]
        actionlevel = []
        positive_goals = goals[0]

        while not positive_goals.issubset(factlevel[i]):
            # actionlevel[i]
            actionlevel.append(
                [a for a in actions
                 if a.positive_preconditions.issubset(factlevel[i])])

            # factlevel[i+1]
            factlevel.append(factlevel[i].union(
                [pre for a in actionlevel[i]
                 for pre in a.add_effects]))
            if factlevel[i+1] == factlevel[i]:
                return float("inf")
            i += 1
        return i


# The following should be visible to the students
# Load some domain and some problem
dwr = "examples/dwr/dwr.pddl"
pb1_dwr = "examples/dwr/pb1.pddl"
pb2_dwr = "examples/dwr/pb2.pddl"

tsp = "examples/tsp/tsp.pddl"
pb1_tsp = "examples/tsp/pb1.pddl"

dinner = "examples/dinner/dinner.pddl"
pb1_dinner = "examples/dinner/pb1.pddl"


def parse_domain_problem(domain, problem):
    parser = PDDLParser()
    parser.parse_domain(domain)
    parser.parse_problem(problem)
    # Grounding process
    actions = []
    for action in parser.actions:
        for act in action.groundify(parser.objects):
            actions.append(act)
    return parser, actions


def test_heuristic(domain, problem, h, expected):
    parser, actions = parse_domain_problem(domain, problem)
    v = h.h(actions, parser.state, (parser.positive_goals, parser.negative_goals))
    print("Expected " + str(expected) + ", got:", str(v) +
          ('. Correct!' if v == expected else '. False!'))


# Apply Hmax to initial states of many problems from many domains
h = MaxHeuristic()
# test_heuristic(dwr, pb1_dwr, h, 6)
# test_heuristic(dwr, pb2_dwr, h, 0)
# test_heuristic(tsp, pb1_tsp, h, 2)
# test_heuristic(dinner, pb1_dinner, h, 1)


class Validator:

    def parse_plan(self, filename):
        with open(filename, 'r') as f:
            plan = []
            for act in f.read().splitlines():
                act = act[1:-1].split()
                plan.append(Action(act[0], tuple(act[1:]), [], [], [], []))
            return plan

    def validate_file(self, domainfile, problemfile, planfile):
        return self.validate_plan(domainfile, problemfile, self.parse_plan(planfile))

    def validate_plan(self, domainfile, problemfile, plan):
        # Parser
        parser = PDDLParser()
        parser.parse_domain(domainfile)
        parser.parse_problem(problemfile)
        # Grounding process
        ground_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects):
                ground_actions.append(act)
        return self.validate(ground_actions, parser.state, (parser.positive_goals, parser.negative_goals), plan)

    # =====================================
    # Params:
    # actions -> list of ground actions
    # initial_state -> initial state of the problem file
    # goals -> tuple with (positive predicates, negative predicates) of the goal
    # plan -> plan parsed from a plan trace
    # =====================================

    # Positive goals and negative goals in a tuple
    def validate(self, actions, initial_state, goals, plan):
        actual_state = initial_state
        for a in actions:
            for p in plan:
                if p.name == a.name:
                    if p.parameters == a.parameters:
                        p.positive_preconditions = a.positive_preconditions
                        p.negative_preconditions = a.negative_preconditions
                        p.add_effects = a.add_effects
                        p.del_effects = a.del_effects

        for p in plan:
            applicable_action = applicable(
                actual_state, (p.positive_preconditions, p.negative_preconditions))
            if applicable_action == True:
                actual_state = apply(
                    actual_state, (p.add_effects, p.del_effects))

        for g in goals:
            if g.issubset(actual_state):
                return True
            else:
                return False


dwr = "examples/dwr/dwr.pddl"
pb1 = "examples/dwr/pb1.pddl"
plan1 = "examples/dwr/dwr_pb1_bfs.plan"
plan2 = "examples/dwr/dwr_pb1_heuristic.plan"
plan_empty = "examples/dwr/empty.plan"
val = Validator()
# print("Expected True, got:", str(val.validate_file(dwr, pb1, plan1)))
# print("Expected True, got:", str(val.validate_file(dwr, pb1, plan2)))
# print("Expected False, got:", str(val.validate_file(dwr, pb1, plan_empty)))


class HeuristicPlanner(PDDLPlanner):

    def __init__(self, heuristic=MaxHeuristic(), verbose=False, collect_stats=False):
        super().__init__(verbose, collect_stats)
        self.h = heuristic

    # -----------------------------------------------
    # Solve
    # -----------------------------------------------

    # =====================================
    # Params:
    # actions -> list of grounded actions
    # state -> initial state of the problem file
    # goals -> tuple with (positive predicates, negative predicates) of the goal
    # =====================================

    # Positive goals and negative goals in a tuple
    def solve(self, actions, state, goals):
        # YOUR CODE HERE
        #raise NotImplementedError()
        Tree = []
        plan = []
        less = Action('take', ['k1', 'cc', 'cb', 'p1', 'l1'], [], [], [], [])
        closed = []
        i = 0
        frontier = queue.PriorityQueue()
        frontier.put((0, state, 0))

        while not (frontier.empty() == True):
            n = frontier.get()
            plan.append(n[1])
            actual_state = n[1]
            closed.append(actual_state)
            if applicable(actual_state, goals):
                return plan

            for a in actions:
                applicable_action = applicable(
                    actual_state, (a.positive_preconditions, a.negative_preconditions))
                if applicable_action == True:
                    resulting_state = apply(
                        actual_state, (a.add_effects, a.del_effects))
                    if resulting_state not in closed:
                        v = n[2] + h.h(actions, resulting_state, goals)
                        frontier.put((v, resulting_state, n[2]+1))

        return plan  # No plan was found


# Student_tests
dwr = "examples/dwr/dwr.pddl"
pb1 = "examples/dwr/pb1.pddl"
pb2 = "examples/dwr/pb2.pddl"

planner = HeuristicPlanner()

plan, time = planner.solve_file(dwr, pb1)
print("Expected 17, got:", str(len(plan)) +
      ('. Correct!' if len(plan) == 17 else '. False!'))

plan, time = planner.solve_file(dwr, pb2)
print("Expected 0, got:", str(len(plan)) +
      ('. Correct!' if len(plan) == 0 else '. False!'))


# Student_tests
# dwr = "examples/dwr/dwr.pddl"
# pb1 = "examples/dwr/pb1.pddl"
# pb2 = "examples/dwr/pb2.pddl"
# planner = HeuristicPlanner()

# plan, time = planner.solve_file(dwr, pb1)
# print("Elapsed time:", str(time) + (' Passed!' if time <= 60.0 else ' Timeout!'))

# plan, time = planner.solve_file(dwr, pb2)
# print("Elapsed time:", str(time) + (' Passed!' if time <= 60.0 else ' Timeout!'))
