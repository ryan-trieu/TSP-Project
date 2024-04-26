import itertools
import heapq
import time

def get_distances_from_file(filename):
    distances = {}
    with open(filename, 'r') as file:
        for line in file:
            city1, city2, distance = line.strip().split()
            distance = int(distance)
            if city1 not in distances:
                distances[city1] = {}
            if city2 not in distances:
                distances[city2] = {}
            distances[city1][city2] = distance
            distances[city2][city1] = distance
    return distances

def get_number_of_locations():
    while True:
        try:
            num_locations = int(input("Enter the number of cities: "))
            if num_locations < 2:
                print("Please enter a number greater than or equal to 2.")
            else:
                return num_locations
        except ValueError:
            print("Invalid input. Please enter a valid integer.")

def get_distances(num_locations):
    distances = {}
    for i in range(num_locations):
        distances[chr(65 + i)] = {}

    print("Enter distances between cities (0 for same city):")
    for i in range(num_locations):
        for j in range(i + 1, num_locations):
            while True:
                try:
                    distance = int(input(f"Distance from {chr(65 + i)} to {chr(65 + j)}: "))
                    if distance < 0:
                        print("Distance cannot be negative.")
                    else:
                        distances[chr(65 + i)][chr(65 + j)] = distance
                        distances[chr(65 + j)][chr(65 + i)] = distance
                        break
                except ValueError:
                    print("Invalid input. Please enter a valid integer.")

    return distances

# Algorithm methods

def calculate_route_length(route, distances):
    length = 0
    for i in range(len(route) - 1):
        length += distances[route[i]][route[i+1]]
    length += distances[route[-1]][route[0]] 
    return length

def brute_force(distances):
    locations = list(distances.keys())
    permutations = itertools.permutations(locations)
    best_route = None
    best_length = float('inf')

    for perm in permutations:
        length = calculate_route_length(perm, distances)

        if length < best_length:
            best_length = length
            best_route = perm

    best_route = list(best_route)
    best_route.append(best_route[0])

    return best_route, best_length



def nearest_neighbor(distances):
    num_cities = len(distances)
    cities = list(distances.keys())
    visited = [False] * num_cities

    route = [cities[0]]
    visited[0] = True
    length = 0

    while len(route) < num_cities:
        current_city = route[-1]
        nearest_city = None
        min_distance = float('inf')

        for city in cities:
            if not visited[cities.index(city)]:
                distance = distances[current_city][city]
                if distance < min_distance:
                    min_distance = distance
                    nearest_city = city

        visited[cities.index(nearest_city)] = True
        route.append(nearest_city)
    
    length = calculate_route_length(route, distances)
    route.append(route[0])

    return route, length

def farthest_neighbor(distances):
    num_cities = len(distances)
    cities = list(distances.keys())
    visited = [False] * num_cities

    route = [cities[0]]
    visited[0] = True
    length = 0

    while len(route) < num_cities:
        current_city = route[-1]
        farthest_city = None
        max_distance = float('-inf')

        for city in cities:
            if not visited[cities.index(city)]:
                distance = distances[current_city][city]
                if distance > max_distance:
                    max_distance = distance
                    farthest_city = city

        visited[cities.index(farthest_city)] = True
        route.append(farthest_city)
    
    length = calculate_route_length(route, distances)
    route.append(route[0])

    return route, length
            
def branch_and_bound(distances):
    import heapq


    def lower_bound(current_route, remaining_cities, current_cost):
        lb = current_cost

        last_city = current_route[-1]
        min_outgoing_edge = min([distances[last_city][city] for city in remaining_cities], default=0)


        minimal_edges = 0
        for city in remaining_cities:
            connected_edges = [distances[city][other] for other in remaining_cities if other != city] + [distances[city][current_route[0]]]
            if connected_edges:  
                minimal_edges += min(connected_edges)

        return lb + min_outgoing_edge + minimal_edges / 2 


    pq = []
    start_city = next(iter(distances))
    initial_route = [start_city]
    remaining_cities = set(distances.keys()) - set(initial_route)
    initial_cost = 0
    initial_bound = lower_bound(initial_route, remaining_cities, initial_cost)
    heapq.heappush(pq, (initial_bound, initial_route, initial_cost, remaining_cities))

    best_route = None
    best_cost = float('inf')

    while pq:
        _, route, current_cost, remaining = heapq.heappop(pq)

        if not remaining:
            total_cost = current_cost + distances[route[-1]][route[0]]
            if total_cost < best_cost:
                best_route = route + [route[0]]
                best_cost = total_cost
        else:
            for next_city in remaining:
                new_route = route + [next_city]
                new_cost = current_cost + distances[route[-1]][next_city]
                new_remaining = remaining - {next_city}
                lb = lower_bound(new_route, new_remaining, new_cost)
                if lb < best_cost:
                    heapq.heappush(pq, (lb, new_route, new_cost, new_remaining))

    return best_route, best_cost







def main():
    filename = input("Enter the filename containing distances: ")
    distances = get_distances_from_file(filename)

    algorithms = {
        1: ("Brute Force", brute_force),
        2: ("Branch and Bound", branch_and_bound),
        3: ("Nearest Neighbor", nearest_neighbor),
        4: ("Farthest Neighbor", farthest_neighbor)
    }

    results = {}

    for alg_num, (alg_name, algorithm) in algorithms.items():
        start_time = time.perf_counter() 
        route, length = algorithm(distances)
        end_time = time.perf_counter()  
        elapsed_time = (end_time - start_time) * 1000  

        results[alg_name] = {
            "Route": route,
            "Length": length,
            "Computation Time": elapsed_time
        }

    # Print results
    print("\n\n")
    for alg_name, result in results.items():
        print(f"{alg_name} Route:", result["Route"])
        print(f"{alg_name} Length:", result["Length"])
        print(f"{alg_name} Computation Time: {result['Computation Time']:.2f} ms")
        print()

if __name__ == "__main__":
    main()
