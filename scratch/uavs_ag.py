from dataclasses import dataclass
import copy
import numpy as np
import random
import math
import matplotlib.pyplot as plt


tx_power = 15  # dbm
f = 6e8
timestep = 5  # seconds
speed = 50
max_users_per_uav = 10 # maximun number of users connected to a given uav
N = 20
n_users = 200
n_services = 5


class UAV:
    """ Model of the UAV device used for optimization
    """

    def __init__(self, id, position, energy, computing, storage) -> None:
        self.id = id
        self.position = position
        self.energy = energy
        self.computing = computing
        self.storage = storage

    def __str__(self) -> str:
        # return f"---\nUAV {self.id}\npos: {self.position}\nenergy remaining: {self.energy}"
        return f"UAV {self.id} pos: {self.position}"

    def set_new_position(self, mov_vector):
        self.position = self.position + mov_vector


def gen_new_position(time):
    # generate a new position based on the time the device has to move and its max speed
    mov_range = speed * time

    signs = np.array([random.choice([-1, 1]), random.choice([-1, 1])])
    mov_vector = np.random.rand(2) * mov_range
    mov_vector *= signs  # randomize the direction of the movement as well
    return mov_vector


@dataclass
class Service:
    computing_cycles: int
    storage: int
    bandwidth: float


class User:
    def __init__(self, id, position) -> None:
        self.position = position
        self.id = id

    def __str__(self) -> str:
        return f"{self.id} {self.position}"


def received_power_calculation(d):
    """  Simple implementaiton of wireless path loss
         since we consider LOS links, I'll keep it fairly simple and increment the complexity as necessary
         power received = power transmitted - path loss
         All units in  dBm
    """
    light_speed = 3e8
    tx_gain = 1
    rx_gain = 1

    # sometimes the simulation thinks the distance is 0 :c
    if d < 1:
        d = 1

    loss = 20 * math.log10(d) + 20 * math.log10(f) + 20 * math.log10(
        4 * math.pi / light_speed) - tx_gain - rx_gain
    return tx_power - loss


def gen_services():
    return Service(np.random.randint(100), np.random.randint(2, 10), np.random.rand() * 10)


def distance(a, b):
    return np.linalg.norm(a - b)


def get_closest_uav(user, uavs):
    index_min = np.argmin([distance(user.position, u.position) for u in uavs])
    return index_min


def scenario_fitness(users, uavs):

    sum_signals = 0

    for user_index, user in enumerate(users):
        serving_uav_index = get_closest_uav(user, uavs)
        serving_uav = uavs[serving_uav_index]

        # below this value, the signal is too weak to really be useful
        received_signal = received_power_calculation(
            distance(user.position, serving_uav.position))

        sum_signals += received_signal

    return sum_signals


def get_coverage(users, uavs, verbose=False):

    connections_matrix = np.zeros((len(uavs), len(users)))

    for user_index, user in enumerate(users):
        serving_uav_index = get_closest_uav(user, uavs)
        serving_uav = uavs[serving_uav_index]

        # below this value, the signal is too weak to really be useful
        reception_threshold = -70
        received_signal = received_power_calculation(
            distance(user.position, serving_uav.position))

        connections_matrix[serving_uav_index, user_index] = 1
        if verbose:
            print(f"User {user} served by {serving_uav}")
            print(f"received power by user {user} = {received_signal}")
            if received_signal < reception_threshold:
                print(f"User is not properly covered")
            else:
                print(f"User is properly covered")

    return connections_matrix


def gen_scenario():
    """ This function is going to return the uav and user objects in their proper locations
    """

    uavs = []
    users = []
    services = []

    initial_energy = 100

    for i in range(n_services):
        services.append(gen_services())

    # generate N uavs to deploy in the scenario
    for i in range(N):
        uavs.append(UAV(i, np.random.randint(
            0, 1000, 2), initial_energy, 100, 100))

    for i in range(n_users):
        users.append(User(i, np.random.randint(0, 1000, 2)))

    return uavs, users, services


def view_scenario(uavs, users):

    plt.figure()
    plt.grid(True)
    plt.title("Scenario Overview")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    for uav in uavs:
        plt.scatter(uav.position[0], uav.position[1], marker="x", color="b")
    for user in users:
        plt.scatter(user.position[0], user.position[1], marker="o", color="r")
    plt.show()


def gen_movement_vector(uavs):
    """ generate movement vector for the scenario based on the UAVs max speed
    """
    new_positions = np.zeros((len(uavs), 2))
    for i, uav in enumerate(uavs):
        new_positions[i] = gen_new_position(timestep)
    return new_positions


def score_solution(users, uavs, mov_vector):
    uavs_tmp = copy.deepcopy(uavs)
    for i, m in enumerate(mov_vector):
        uavs_tmp[i].set_new_position(m)
    return scenario_fitness(users, uavs_tmp)


def view_solution(users, uavs, mov_vector):

    plt.figure()
    plt.grid(True)
    plt.title("Scenario Overview")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")

    for uav in uavs:
        plt.scatter(uav.position[0], uav.position[1], marker="x", color="b")
    for user in users:
        plt.scatter(user.position[0], user.position[1], marker="o", color="r")

    for uav, m in zip(uavs, mov_vector):
        plt.arrow(uav.position[0], uav.position[1], + m[0],
                  m[1], fc="r", ec="r", head_width=1, head_length=0.1)

    plt.show()


def sort_solutions(users, uavs, solutions):
    return sorted(solutions, key=lambda x: score_solution(users, uavs, x), reverse=True)


def mutate(uavs, solutions):
    rate = 0.1
    elitism = True

    for i, s in enumerate(solutions):
        if np.random.rand() < rate and (elitism and i != 0):
            solutions[i] = gen_movement_vector(uavs)

    return solutions


def crossover(solutions):
    rate = 0.2
    for i, s in enumerate(solutions):
        if i % 2 == 0 and i > 0:
            if np.random.rand() < rate:
                try:
                    for g_index, gene in enumerate(s):
                        solutions[i][g_index], solutions[i +
                                                         1][g_index] = solutions[i + 1][g_index], solutions[i][g_index]
                except IndexError:
                    pass
    return solutions


def prune(uavs, solutions):
    rate = 0.5

    for i, s in enumerate(solutions):
        if i > rate * len(solutions):
            solutions[i] = gen_movement_vector(uavs)
    return solutions


def gen_solutions(users, uavs):
    n_solutions = 100
    rounds = 50

    solutions = []
    fitness = []

    for s in range(n_solutions):
        mov_vector = gen_movement_vector(uavs)
        solutions.append(mov_vector)

    for r in range(rounds):

        print(f"beginnig round {r}")
        solutions = sort_solutions(users, uavs, solutions)
        solutions = mutate(uavs, solutions)
        solutions = crossover(solutions)
        solutions = prune(uavs, solutions)

        fitness.append(score_solution(users, uavs, solutions[0]))

    # escrever solução num arquivo
    plt.plot(fitness)

    view_solution(users, uavs, solutions[0])


def view_connections_matrix(cm):
    plt.figure()
    plt.title("Connections Matrix")
    plt.imshow(cm, interpolation="none")
    plt.show()


def main():

    uavs, users, services = gen_scenario()

    gen_solutions(users, uavs)


if __name__ == "__main__":
    main()
