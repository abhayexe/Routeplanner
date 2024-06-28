from flask import Flask, render_template, request, redirect, url_for
import folium
import openrouteservice as ors
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

app = Flask(__name__)

client = ors.Client(key='5b3ce3597851110001cf62482f14c4be0a8c405a9d7316d5cf55a965')

def get_coordinates(place_names, client):
    coordinates = []
    for place in place_names:
        geocode = client.pelias_search(text=place)
        if geocode and 'features' in geocode and len(geocode['features']) > 0:
            coords = geocode['features'][0]['geometry']['coordinates']
            coordinates.append(coords)
        else:
            print(f"Could not find coordinates for {place}")
    return coordinates

def compute_euclidean_distance_matrix(locations):
    distances = np.zeros((len(locations), len(locations)))
    for i, from_node in enumerate(locations):
        for j, to_node in enumerate(locations):
            if i != j:
                distances[i][j] = np.linalg.norm(np.array(from_node) - np.array(to_node))
    return distances

def create_data_model(distance_matrix):
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def solve_tsp(distance_matrix):
    data = create_data_model(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        index = routing.Start(0)
        plan_output = []
        while not routing.IsEnd(index):
            plan_output.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        return plan_output
    else:
        return None

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        places = request.form.getlist('places')
        start_location = request.form['start_location']
        
        coords = get_coordinates(places, client)
        if not coords:
            return "No valid coordinates found. Please try again."

        vehicle_start = get_coordinates([start_location], client)
        if not vehicle_start:
            return "Could not find coordinates for the starting location. Please try again."
        vehicle_start = vehicle_start[0]

        coords.insert(0, vehicle_start)

        distance_matrix = compute_euclidean_distance_matrix(coords)
        route = solve_tsp(distance_matrix)
        if route is None:
            return "No solution found. Please try again."

        route_coords = [coords[i] for i in route]

        m = folium.Map(location=list(reversed(vehicle_start)), tiles="cartodbpositron", zoom_start=14)
        for coord in coords:
            folium.Marker(location=list(reversed(coord))).add_to(m)
        folium.Marker(location=list(reversed(vehicle_start)), icon=folium.Icon(color="red")).add_to(m)

        for i in range(len(route_coords) - 1):
            start = route_coords[i]
            end = route_coords[i + 1]
            directions = client.directions(coordinates=[start, end], profile='driving-car', format='geojson')
            folium.PolyLine(locations=[list(reversed(coord)) for coord in directions['features'][0]['geometry']['coordinates']], color='blue').add_to(m)

        directions = client.directions(coordinates=[route_coords[-1], route_coords[0]], profile='driving-car', format='geojson')
        folium.PolyLine(locations=[list(reversed(coord)) for coord in directions['features'][0]['geometry']['coordinates']], color='blue').add_to(m)

        m.save('static/tsp_route.html')
        return redirect(url_for('map'))

    return render_template('index.html')

@app.route('/map')
def map():
    return render_template('map.html')

if __name__ == '__main__':
    app.run(debug=True)
