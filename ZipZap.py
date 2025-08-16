import sys
import json
import math
import random
import datetime
import pyttsx3
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QLineEdit, QPushButton, QComboBox, QMessageBox, 
                             QListWidget, QGraphicsView, QGraphicsScene, QGroupBox,
                             QDoubleSpinBox, QSpinBox, QCheckBox, QTabWidget, QFileDialog,
                             QSizePolicy, QSpacerItem, QCompleter)
from PyQt5.QtCore import Qt, QUrl, QPropertyAnimation, QPointF, QStringListModel
from PyQt5.QtGui import QColor, QPen, QBrush, QFont, QIcon
from PyQt5.QtWebEngineWidgets import QWebEngineView
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from geopy.distance import geodesic
from geopy.geocoders import Nominatim

class CityGraph:
    """Graph class to represent cities and roads using adjacency list"""
    def __init__(self):
        self.adj_list = {}
        self.city_coords = {}  # Stores city names and their coordinates (lat, lon)
        
    def add_city(self, city_name, lat=None, lon=None):
        """Add a new city to the graph"""
        if city_name not in self.adj_list:
            self.adj_list[city_name] = []
            self.city_coords[city_name] = (lat, lon)
            return True
        return False
    
    def remove_city(self, city_name):
        """Remove a city from the graph"""
        if city_name in self.adj_list:
            # Remove all roads connected to this city
            for connected_cities in self.adj_list.values():
                connected_cities[:] = [road for road in connected_cities if road[0] != city_name]
            
            del self.adj_list[city_name]
            del self.city_coords[city_name]
            return True
        return False
    
    def haversine_distance(self, coord1, coord2):
        """Calculate the distance between two coordinates using Haversine formula"""
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371  # Radius of Earth in kilometers
        return c * r
    
    def add_road(self, city1, city2, distance=None, weighted=True):
        """Add a road between two cities"""
        if city1 not in self.adj_list or city2 not in self.adj_list:
            return False
        
        # Calculate real distance if not provided and coordinates exist
        if distance is None:
            coord1 = self.city_coords.get(city1, (None, None))
            coord2 = self.city_coords.get(city2, (None, None))
            
            if coord1[0] is not None and coord2[0] is not None:
                distance = self.haversine_distance(coord1, coord2)
            else:
                # If no coordinates, use default distance of 1 for BFS
                distance = 1 if weighted else 1
        
        # Check if road already exists
        for road in self.adj_list[city1]:
            if road[0] == city2:
                return False
                
        # Add bidirectional road
        self.adj_list[city1].append((city2, distance))
        self.adj_list[city2].append((city1, distance))
        return True
    
    def remove_road(self, city1, city2):
        """Remove a road between two cities"""
        if city1 in self.adj_list and city2 in self.adj_list:
            # Remove from city1's connections
            self.adj_list[city1] = [road for road in self.adj_list[city1] if road[0] != city2]
            # Remove from city2's connections
            self.adj_list[city2] = [road for road in self.adj_list[city2] if road[0] != city1]
            return True
        return False
    
    def get_cities(self):
        """Return list of all cities"""
        return list(self.adj_list.keys())
    
    def get_roads(self, city):
        """Return all roads connected to a city"""
        return self.adj_list.get(city, [])
    
    def get_road_distance(self, city1, city2):
        """Get distance between two connected cities"""
        for road in self.adj_list.get(city1, []):
            if road[0] == city2:
                return road[1]
        return None
    
    def bfs_shortest_path(self, start, end):
        """Find shortest path using BFS (unweighted)"""
        if start not in self.adj_list or end not in self.adj_list:
            return None, None, None
            
        visited = {city: False for city in self.adj_list}
        parent = {city: None for city in self.adj_list}
        queue = deque([start])
        visited[start] = True
        steps = 0
        
        while queue:
            current = queue.popleft()
            
            if current == end:
                break
                
            for neighbor, _ in self.adj_list[current]:
                if not visited[neighbor]:
                    visited[neighbor] = True
                    parent[neighbor] = current
                    queue.append(neighbor)
            steps += 1
        
        # Reconstruct path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        
        if path[0] == start:
            # Calculate total distance (count each edge as 1)
            distance = len(path) - 1
            return path, distance, steps
        else:
            return None, None, None
    
    def dijkstra_shortest_path(self, start, end):
        """Find shortest path using Dijkstra's algorithm (weighted)"""
        if start not in self.adj_list or end not in self.adj_list:
            return None, None, None
            
        distances = {city: float('inf') for city in self.adj_list}
        distances[start] = 0
        parent = {city: None for city in self.adj_list}
        visited = set()
        steps = 0
        
        while True:
            # Find unvisited city with smallest distance
            unvisited = {city: dist for city, dist in distances.items() if city not in visited}
            if not unvisited:
                break
                
            current = min(unvisited, key=unvisited.get)
            if current == end:
                break
                
            visited.add(current)
            
            for neighbor, distance in self.adj_list[current]:
                if neighbor not in visited:
                    new_distance = distances[current] + distance
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        parent[neighbor] = current
                steps += 1
        
        # Reconstruct path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        
        if path[0] == start:
            return path, distances[end], steps
        else:
            return None, None, None
    
    def to_dict(self):
        """Convert graph to dictionary for JSON serialization"""
        return {
            'cities': {city: {'lat': coords[0], 'lon': coords[1]} for city, coords in self.city_coords.items()},
            'roads': {city: roads for city, roads in self.adj_list.items()}
        }
    
    @classmethod
    def from_dict(cls, data):
        """Create graph from dictionary"""
        graph = cls()
        for city, coords in data['cities'].items():
            graph.add_city(city, coords['lat'], coords['lon'])
        
        for city, roads in data['roads'].items():
            for road in roads:
                neighbor, distance = road
                # Ensure we don't add duplicates
                if neighbor in graph.adj_list and city not in [r[0] for r in graph.adj_list[neighbor]]:
                    graph.add_road(city, neighbor, distance)
        return graph

class MapView(QWebEngineView):
    """Interactive map view using OpenStreetMap with enhanced features"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)
        self.loaded = False
        self.html_template = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Route Map</title>
            <meta charset="utf-8" />
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
            <style>
                body { margin: 0; padding: 0; }
                #map { height: 100vh; width: 100%; }
                .start-icon { 
                    background-color: green;
                    border-radius: 50%;
                    width: 20px;
                    height: 20px;
                    text-align: center;
                    color: white;
                    font-weight: bold;
                    line-height: 20px;
                }
                .end-icon { 
                    background-color: red;
                    border-radius: 50%;
                    width: 20px;
                    height: 20px;
                    text-align: center;
                    color: white;
                    font-weight: bold;
                    line-height: 20px;
                }
                .waypoint-icon { 
                    background-color: blue;
                    border-radius: 50%;
                    width: 15px;
                    height: 15px;
                    text-align: center;
                    color: white;
                    font-weight: bold;
                    line-height: 15px;
                }
                .distance-info {
                    background: white;
                    padding: 8px;
                    border-radius: 5px;
                    box-shadow: 0 0 10px rgba(0,0,0,0.2);
                    font-size: 16px;
                    font-weight: bold;
                    border: 2px solid #4a6fa5;
                }
                .city-label {
                    font-weight: bold;
                    font-size: 14px;
                    color: #333;
                    text-shadow: -1px -1px 0 #fff, 1px -1px 0 #fff, -1px 1px 0 #fff, 1px 1px 0 #fff;
                }
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map;
                var routeLayer;
                var markers = [];
                
                function initMap() {
                    map = L.map('map').setView([17.3850, 78.4867], 12);
                    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                    }).addTo(map);
                    window.mapInitialized = true;
                }
                
                function showRoute(pathCoords, cityNames) {
                    if (!window.mapInitialized) initMap();
                    
                    // Clear previous layers
                    if (routeLayer) {
                        map.removeLayer(routeLayer);
                    }
                    markers.forEach(marker => map.removeLayer(marker));
                    markers = [];
                    
                    if (!pathCoords || pathCoords.length < 2) return;
                    
                    // Draw the route
                    routeLayer = L.polyline(pathCoords, {
                        color: '#4a6fa5',
                        weight: 6,
                        opacity: 0.8,
                        dashArray: '10, 5'
                    }).addTo(map);
                    
                    // Add markers for all points with labels
                    pathCoords.forEach((coord, index) => {
                        let iconClass, iconHtml;
                        if (index === 0) {
                            iconClass = 'start-icon';
                            iconHtml = 'S';
                        } else if (index === pathCoords.length - 1) {
                            iconClass = 'end-icon';
                            iconHtml = 'E';
                        } else {
                            iconClass = 'waypoint-icon';
                            iconHtml = (index).toString();
                        }
                        
                        let marker = L.marker(coord, {
                            icon: L.divIcon({
                                className: iconClass,
                                html: iconHtml
                            })
                        }).addTo(map);
                        
                        // Add city name label next to marker
                        if (cityNames && cityNames[index]) {
                            // Add permanent label
                            L.marker(coord, {
                                icon: L.divIcon({
                                    className: 'city-label',
                                    html: cityNames[index],
                                    iconSize: [100, 20]
                                }),
                                zIndexOffset: 1000
                            }).addTo(map);
                            
                            // Add popup with more info
                            marker.bindPopup(`
                                <b>${cityNames[index]}</b><br>
                                ${index === 0 ? 'Start Point' : index === pathCoords.length - 1 ? 'End Point' : 'Waypoint ' + index}<br>
                                Lat: ${coord[0].toFixed(6)}<br>
                                Lon: ${coord[1].toFixed(6)}
                            `);
                        }
                        
                        markers.push(marker);
                    });
                    
                    // Fit bounds to show entire route with padding
                    map.fitBounds(routeLayer.getBounds(), {padding: [50, 50]});
                    
                    // Calculate and display total distance
                    let totalDistance = 0;
                    for (let i = 0; i < pathCoords.length - 1; i++) {
                        totalDistance += map.distance(pathCoords[i], pathCoords[i+1]) / 1000; // Convert to km
                    }
                    
                    // Add distance info to the map
                    let distanceInfo = L.control({position: 'bottomleft'});
                    distanceInfo.onAdd = function(map) {
                        this._div = L.DomUtil.create('div', 'distance-info');
                        this.update();
                        return this._div;
                    };
                    distanceInfo.update = function() {
                        this._div.innerHTML = `
                            <b>Total Distance:</b> ${totalDistance.toFixed(2)} km<br>
                            <b>Route Points:</b> ${pathCoords.length}
                        `;
                    };
                    distanceInfo.addTo(map);
                    
                    // Store reference to remove later
                    if (window.distanceControl) {
                        map.removeControl(window.distanceControl);
                    }
                    window.distanceControl = distanceInfo;
                }
                
                // Initialize map when page loads
                document.addEventListener('DOMContentLoaded', initMap);
            </script>
        </body>
        </html>
        """
        self.setHtml(self.html_template)
        
    def show_route(self, path_coords, city_names=None):
        """Display a route on the map with markers and distance"""
        if not path_coords or len(path_coords) < 2:
            return
            
        # Convert coordinates to list of lists for JavaScript
        coords_list = [[lat, lon] for lat, lon in path_coords]
        
        # Create the JavaScript call
        script = f"showRoute({coords_list}, {city_names or []});"
        
        # Run the script after ensuring the map is loaded
        self.page().runJavaScript(script)

class GraphCanvas(FigureCanvas):
    """Canvas for displaying the graph visualization"""
    def __init__(self, parent=None, width=5, height=5, dpi=100):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
        self.setParent(parent)
        self.graph = None
        self.path = None
        self.start_city = None
        self.end_city = None
    
    def draw_graph(self, graph, path=None, start_city=None, end_city=None):
        """Draw the graph with optional path highlighting"""
        self.graph = graph
        self.path = path
        self.start_city = start_city
        self.end_city = end_city
        
        if not graph.get_cities():
            self.ax.clear()
            self.ax.text(0.5, 0.5, 'No cities in graph', ha='center', va='center')
            self.draw()
            return
            
        # Create networkx graph
        G = nx.Graph()
        
        # Add nodes
        for city in graph.get_cities():
            G.add_node(city)
            
        # Add edges
        for city in graph.get_cities():
            for neighbor, distance in graph.get_roads(city):
                G.add_edge(city, neighbor, weight=distance)
        
        # Choose layout
        pos = nx.spring_layout(G)
        
        # Draw the graph
        self.ax.clear()
        
        # Draw all nodes
        nx.draw_networkx_nodes(G, pos, node_size=700, ax=self.ax)
        
        # Draw all edges
        nx.draw_networkx_edges(G, pos, ax=self.ax)
        
        # Highlight path edges if provided
        if path and len(path) >= 2:
            path_edges = list(zip(path[:-1], path[1:]))
            nx.draw_networkx_edges(G, pos, edgelist=path_edges, width=3, edge_color='r', ax=self.ax)
        
        # Highlight start and end nodes
        node_colors = []
        for node in G.nodes():
            if node == start_city:
                node_colors.append('green')
            elif node == end_city:
                node_colors.append('red')
            else:
                node_colors.append('skyblue')
        
        nx.draw_networkx_nodes(G, pos, node_color=node_colors, node_size=700, ax=self.ax)
        
        # Draw labels
        nx.draw_networkx_labels(G, pos, ax=self.ax)
        
        # Draw edge weights
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, ax=self.ax)
        
        self.ax.set_axis_off()
        self.draw()

class RoutePlannerApp(QMainWindow):
    """Main application window for the route planner"""
    def __init__(self):
        super().__init__()
        self.graph = CityGraph()
        self.geolocator = Nominatim(user_agent="quickpath_logistics")
        self.last_path = None
        self.init_ui()
        self.load_sample_data()
        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('QuickPath Logistics - Route Planning System')
        self.setGeometry(100, 100, 1200, 800)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(15)
        
        # Left panel - controls
        left_panel = QVBoxLayout()
        left_panel.setSpacing(15)
        
        # City management group
        city_group = QGroupBox("City Management")
        city_layout = QVBoxLayout()
        city_layout.setSpacing(8)
        
        self.city_list = QListWidget()
        self.city_list.setMinimumHeight(150)
        city_layout.addWidget(QLabel("Cities:"))
        city_layout.addWidget(self.city_list)
        
        self.city_name_input = QLineEdit()
        self.city_name_input.setPlaceholderText("City name")
        city_layout.addWidget(self.city_name_input)
        
        coord_layout = QHBoxLayout()
        self.lat_input = QDoubleSpinBox()
        self.lat_input.setRange(-90, 90)
        self.lat_input.setDecimals(6)
        self.lat_input.setPrefix("Lat: ")
        self.lat_input.setValue(17.3850)
        
        self.lon_input = QDoubleSpinBox()
        self.lon_input.setRange(-180, 180)
        self.lon_input.setDecimals(6)
        self.lon_input.setPrefix("Lon: ")
        self.lon_input.setValue(78.4867)
        
        coord_layout.addWidget(self.lat_input)
        coord_layout.addWidget(self.lon_input)
        city_layout.addLayout(coord_layout)
        
        btn_layout = QHBoxLayout()
        self.add_city_btn = QPushButton("Add")
        self.add_city_btn.clicked.connect(self.add_city)
        self.update_city_btn = QPushButton("Update")
        self.update_city_btn.clicked.connect(self.update_city)
        self.update_city_btn.setEnabled(False)
        self.remove_city_btn = QPushButton("Remove")
        self.remove_city_btn.clicked.connect(self.remove_city)
        
        btn_layout.addWidget(self.add_city_btn)
        btn_layout.addWidget(self.update_city_btn)
        btn_layout.addWidget(self.remove_city_btn)
        city_layout.addLayout(btn_layout)
        
        self.fetch_coords_btn = QPushButton("Fetch Coordinates")
        self.fetch_coords_btn.clicked.connect(self.fetch_coordinates)
        city_layout.addWidget(self.fetch_coords_btn)
        
        city_group.setLayout(city_layout)
        left_panel.addWidget(city_group)
        
        # Road management group
        road_group = QGroupBox("Road Management")
        road_layout = QVBoxLayout()
        road_layout.setSpacing(8)
        
        self.from_city_combo = QComboBox()
        self.to_city_combo = QComboBox()
        road_layout.addWidget(QLabel("From City:"))
        road_layout.addWidget(self.from_city_combo)
        road_layout.addWidget(QLabel("To City:"))
        road_layout.addWidget(self.to_city_combo)
        
        self.distance_input = QDoubleSpinBox()
        self.distance_input.setRange(0, 9999)
        self.distance_input.setSuffix(" km")
        self.distance_input.setValue(10)
        self.distance_input.setEnabled(False)
        
        self.auto_distance_check = QCheckBox("Calculate distance automatically")
        self.auto_distance_check.setChecked(True)
        self.auto_distance_check.stateChanged.connect(self.toggle_distance_input)
        
        road_layout.addWidget(self.auto_distance_check)
        road_layout.addWidget(self.distance_input)
        
        road_btn_layout = QHBoxLayout()
        self.add_road_btn = QPushButton("Add Road")
        self.add_road_btn.clicked.connect(self.add_road)
        self.remove_road_btn = QPushButton("Remove Road")
        self.remove_road_btn.clicked.connect(self.remove_road)
        road_btn_layout.addWidget(self.add_road_btn)
        road_btn_layout.addWidget(self.remove_road_btn)
        road_layout.addLayout(road_btn_layout)
        
        road_group.setLayout(road_layout)
        left_panel.addWidget(road_group)
        
        # Path finding group
        path_group = QGroupBox("Path Finding")
        path_layout = QVBoxLayout()
        path_layout.setSpacing(8)
        
        # Start/End city inputs with autocomplete
        self.start_city_combo = QComboBox()
        self.end_city_combo = QComboBox()
        
        # Setup autocomplete for city inputs
        self.setup_autocomplete()
        
        path_layout.addWidget(QLabel("Start City:"))
        path_layout.addWidget(self.start_city_combo)
        path_layout.addWidget(QLabel("End City:"))
        path_layout.addWidget(self.end_city_combo)
        
        self.algorithm_combo = QComboBox()
        self.algorithm_combo.addItem("Dijkstra's Algorithm (Weighted)", "dijkstra")
        self.algorithm_combo.addItem("BFS (Unweighted)", "bfs")
        self.algorithm_combo.addItem("Compare Both Algorithms", "compare")
        path_layout.addWidget(QLabel("Algorithm:"))
        path_layout.addWidget(self.algorithm_combo)
        
        # Vehicle type selection
        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItem("Car (15 km/l)", "car")
        self.vehicle_combo.addItem("Bike (50 km/l)", "bike")
        self.vehicle_combo.addItem("Truck (5 km/l)", "truck")
        path_layout.addWidget(QLabel("Vehicle Type:"))
        path_layout.addWidget(self.vehicle_combo)
        
        # Fuel cost inputs
        fuel_layout = QHBoxLayout()
        self.fuel_cost_input = QDoubleSpinBox()
        self.fuel_cost_input.setRange(0, 200)
        self.fuel_cost_input.setPrefix("₹")
        self.fuel_cost_input.setSuffix(" per liter")
        self.fuel_cost_input.setValue(110)
        fuel_layout.addWidget(QLabel("Fuel Cost:"))
        fuel_layout.addWidget(self.fuel_cost_input)
        path_layout.addLayout(fuel_layout)
        
        self.find_path_btn = QPushButton("Find Shortest Path")
        self.find_path_btn.clicked.connect(self.find_shortest_path)
        path_layout.addWidget(self.find_path_btn)
        
        # Results section
        results_group = QGroupBox("Results")
        results_layout = QVBoxLayout()
        
        # Algorithm comparison results
        self.comparison_result = QLabel()
        self.comparison_result.setWordWrap(True)
        self.comparison_result.setStyleSheet("font-weight: bold; color: #8e44ad;")
        self.comparison_result.hide()
        results_layout.addWidget(self.comparison_result)
        
        self.path_result = QLabel()
        self.path_result.setWordWrap(True)
        self.path_result.setStyleSheet("font-weight: bold;")
        results_layout.addWidget(self.path_result)
        
        self.distance_result = QLabel()
        self.distance_result.setWordWrap(True)
        results_layout.addWidget(self.distance_result)
        
        # Fuel calculation results
        self.fuel_result = QLabel()
        self.fuel_result.setWordWrap(True)
        results_layout.addWidget(self.fuel_result)
        
        self.fuel_cost_result = QLabel()
        self.fuel_cost_result.setWordWrap(True)
        results_layout.addWidget(self.fuel_cost_result)
        
        self.traffic_label = QLabel("🚦 Traffic: Not calculated")
        self.traffic_label.setStyleSheet("color: #d35400; font-weight: bold;")
        results_layout.addWidget(self.traffic_label)
        
        self.eco_label = QLabel("🌱 Eco Score: Not calculated")
        self.eco_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        results_layout.addWidget(self.eco_label)
        
        self.ai_label = QLabel("🤖 AI Suggestion: Try finding a route to see suggestions")
        self.ai_label.setWordWrap(True)
        self.ai_label.setStyleSheet("color: #2980b9; font-style: italic;")
        results_layout.addWidget(self.ai_label)
        
        self.voice_btn = QPushButton("🔊 Voice Navigation")
        self.voice_btn.clicked.connect(self.enable_voice_navigation)
        results_layout.addWidget(self.voice_btn)
        
        results_group.setLayout(results_layout)
        path_layout.addWidget(results_group)
        
        path_group.setLayout(path_layout)
        left_panel.addWidget(path_group)
        
        # File operations
        file_group = QGroupBox("File Operations")
        file_layout = QHBoxLayout()
        
        self.save_btn = QPushButton("Save Graph")
        self.save_btn.clicked.connect(self.save_graph)
        self.load_btn = QPushButton("Load Graph")
        self.load_btn.clicked.connect(self.load_graph)
        
        file_layout.addWidget(self.save_btn)
        file_layout.addWidget(self.load_btn)
        file_group.setLayout(file_layout)
        left_panel.addWidget(file_group)
        
        # Add stretch to push everything up
        left_panel.addStretch()
        
        # Right panel - visualization
        right_panel = QVBoxLayout()
        
        # Tab widget for different views
        self.tab_widget = QTabWidget()
        
        # Graph visualization tab
        self.graph_tab = QWidget()
        self.graph_layout = QVBoxLayout(self.graph_tab)
        self.graph_canvas = GraphCanvas(self.graph_tab)
        self.graph_layout.addWidget(self.graph_canvas)
        self.tab_widget.addTab(self.graph_tab, "Graph View")
        
        # Map view tab
        self.map_tab = QWidget()
        self.map_layout = QVBoxLayout(self.map_tab)
        self.map_view = MapView(self.map_tab)
        self.map_layout.addWidget(self.map_view)
        self.tab_widget.addTab(self.map_tab, "Map View")
        
        right_panel.addWidget(self.tab_widget)
        
        # Add panels to main layout
        main_layout.addLayout(left_panel, 40)
        main_layout.addLayout(right_panel, 60)
        
        # Apply modern styling
        self.setStyleSheet("""
            QMainWindow {
                background: #f5f7fa;
            }
            QGroupBox {
                background: white;
                border: 1px solid #dcdcdc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }
            QPushButton {
                background: #4a6fa5;
                color: white;
                border-radius: 4px;
                padding: 8px;
                border: none;
                font-weight: bold;
                min-width: 80px;
            }
            QPushButton:hover {
                background: #3a5a80;
            }
            QPushButton:pressed {
                background: #2a4a70;
            }
            QLineEdit, QComboBox, QListWidget {
                border: 1px solid #dcdcdc;
                border-radius: 4px;
                padding: 5px;
                min-height: 25px;
            }
            QLabel {
                color: #333333;
            }
            QTabWidget::pane {
                border: 1px solid #dcdcdc;
                border-radius: 4px;
            }
        """)
        
        # Connect city selection event
        self.city_list.itemSelectionChanged.connect(self.on_city_selected)
        
        # Update UI
        self.update_city_lists()
    
    def setup_autocomplete(self):
        """Setup autocomplete for city selection comboboxes"""
        cities = self.graph.get_cities()
        completer = QCompleter(cities)
        completer.setCaseSensitivity(Qt.CaseInsensitive)
        completer.setFilterMode(Qt.MatchContains)
        
        # Apply completer to start and end city comboboxes
        for combo in [self.start_city_combo, self.end_city_combo]:
            combo.setEditable(True)
            combo.setCompleter(completer)
    
    def toggle_distance_input(self, state):
        """Enable/disable distance input based on checkbox"""
        self.distance_input.setEnabled(not bool(state))
    
    def update_city_lists(self):
        """Update all city dropdown lists and list widget"""
        cities = self.graph.get_cities()
        
        # Update city list widget
        self.city_list.clear()
        self.city_list.addItems(cities)
        
        # Update combo boxes
        for combo in [self.from_city_combo, self.to_city_combo, 
                     self.start_city_combo, self.end_city_combo]:
            current = combo.currentText()
            combo.clear()
            combo.addItems(cities)
            if current in cities:
                combo.setCurrentText(current)
        
        # Update autocomplete
        self.setup_autocomplete()
        
        # Redraw graph
        self.graph_canvas.draw_graph(self.graph)
    
    def on_city_selected(self):
        """Handle city selection in the list"""
        selected = self.city_list.currentItem()
        if selected:
            city = selected.text()
            coords = self.graph.city_coords.get(city, (None, None))
            
            self.city_name_input.setText(city)
            self.lat_input.setValue(coords[0] if coords[0] is not None else 0)
            self.lon_input.setValue(coords[1] if coords[1] is not None else 0)
            
            self.update_city_btn.setEnabled(True)
        else:
            self.city_name_input.clear()
            self.update_city_btn.setEnabled(False)
    
    def add_city(self):
        """Add a new city to the graph"""
        city = self.city_name_input.text().strip()
        lat = self.lat_input.value()
        lon = self.lon_input.value()
        
        if not city:
            QMessageBox.warning(self, "Error", "Please enter a city name")
            return
            
        # Validate coordinates
        if lat == 0 or lon == 0:
            reply = QMessageBox.question(self, "Confirm", 
                                        "Coordinates not set. Do you want to set default Hyderabad coordinates?",
                                        QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                # Set default Hyderabad coordinates (center point)
                lat = 17.3850
                lon = 78.4867
                self.lat_input.setValue(lat)
                self.lon_input.setValue(lon)
            else:
                QMessageBox.warning(self, "Warning", "Road distances may not be accurate without proper coordinates")
        
        if self.graph.add_city(city, lat, lon):
            self.update_city_lists()
            self.city_name_input.clear()
            QMessageBox.information(self, "Success", f"City '{city}' added successfully")
        else:
            QMessageBox.warning(self, "Error", f"City '{city}' already exists")
    
    def update_city(self):
        """Update selected city's coordinates"""
        selected = self.city_list.currentItem()
        if not selected:
            return
            
        old_city = selected.text()
        new_city = self.city_name_input.text().strip()
        lat = self.lat_input.value()
        lon = self.lon_input.value()
        
        if not new_city:
            QMessageBox.warning(self, "Error", "Please enter a city name")
            return
            
        # If name changed, we need to remove and re-add
        if old_city != new_city:
            # Get all roads connected to this city
            connected_roads = []
            for neighbor, roads in self.graph.adj_list.items():
                for road in roads:
                    if road[0] == old_city:
                        connected_roads.append((neighbor, road[1]))
            
            # Remove the city
            self.graph.remove_city(old_city)
            
            # Add with new name
            self.graph.add_city(new_city, lat, lon)
            
            # Re-add all roads
            for neighbor, distance in connected_roads:
                self.graph.add_road(new_city, neighbor, distance)
        else:
            # Just update coordinates
            self.graph.city_coords[old_city] = (lat, lon)
        
        self.update_city_lists()
        QMessageBox.information(self, "Success", f"City '{new_city}' updated successfully")
    
    def remove_city(self):
        """Remove selected city from the graph"""
        selected = self.city_list.currentItem()
        if not selected:
            return
            
        city = selected.text()
        
        reply = QMessageBox.question(self, "Confirm", 
                                    f"Remove city '{city}' and all connected roads?", 
                                    QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.graph.remove_city(city)
            self.update_city_lists()
            QMessageBox.information(self, "Success", f"City '{city}' removed successfully")
    
    def fetch_coordinates(self):
        """Fetch coordinates for the current city using Nominatim"""
        city = self.city_name_input.text().strip()
        if not city:
            QMessageBox.warning(self, "Error", "Please enter a city name first")
            return
            
        try:
            # Add ", Hyderabad" to help with disambiguation
            location = self.geolocator.geocode(f"{city}, Hyderabad, India")
            if location:
                self.lat_input.setValue(location.latitude)
                self.lon_input.setValue(location.longitude)
                QMessageBox.information(self, "Success", 
                                       f"Coordinates found for {city}: {location.latitude}, {location.longitude}")
            else:
                QMessageBox.warning(self, "Error", f"Could not find coordinates for {city}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to fetch coordinates: {str(e)}")
    
    def add_road(self):
        """Add a road between two cities"""
        from_city = self.from_city_combo.currentText()
        to_city = self.to_city_combo.currentText()
        
        if not from_city or not to_city:
            QMessageBox.warning(self, "Error", "Please select both cities")
            return
            
        if from_city == to_city:
            QMessageBox.warning(self, "Error", "Cannot connect a city to itself")
            return
            
        distance = None
        if not self.auto_distance_check.isChecked():
            distance = self.distance_input.value()
        
        if self.graph.add_road(from_city, to_city, distance):
            self.update_city_lists()
            QMessageBox.information(self, "Success", 
                                  f"Road between '{from_city}' and '{to_city}' added successfully")
        else:
            QMessageBox.warning(self, "Error", 
                              f"Road between '{from_city}' and '{to_city}' already exists")
    
    def remove_road(self):
        """Remove a road between two cities"""
        from_city = self.from_city_combo.currentText()
        to_city = self.to_city_combo.currentText()
        
        if not from_city or not to_city:
            QMessageBox.warning(self, "Error", "Please select both cities")
            return
            
        if self.graph.remove_road(from_city, to_city):
            self.update_city_lists()
            QMessageBox.information(self, "Success", 
                                  f"Road between '{from_city}' and '{to_city}' removed successfully")
        else:
            QMessageBox.warning(self, "Error", 
                              f"No road exists between '{from_city}' and '{to_city}'")
    
    def get_traffic_data(self, route_coords):
        """Simulate traffic data (in a real app, use API like Google Maps)"""
        # Peak hours simulation (8-10am, 5-8pm)
        now = datetime.datetime.now().time()
        is_peak = (8 <= now.hour < 10) or (17 <= now.hour < 20)
        
        # Return traffic multiplier (1.0 = normal, 1.5 = heavy traffic)
        return 1.5 if is_peak else 1.0
    
    def calculate_eco_score(self, route):
        """Calculate how eco-friendly a route is"""
        if not route or len(route) < 2:
            return 0
            
        distance = sum(self.graph.get_road_distance(route[i], route[i+1]) 
                   for i in range(len(route)-1))
        
        # Simulate elevation data (flat routes are more eco-friendly)
        score = 100 - (distance * 0.2)  
        return min(100, max(0, int(score)))
    
    def generate_ai_suggestion(self):
        """Simulate AI suggesting alternative routes"""
        alternatives = [
            "Consider leaving 30 mins earlier to avoid traffic",
            "This route has 3 charging stations for EVs",
            "Alternative scenic route available (+12% time)",
            "Fuel-efficient route available (saves ₹85)",
            "Current route has moderate traffic during this time",
            "Try our new eco-friendly route option",
            "This path has the lowest elevation changes",
            "Consider carpooling to reduce costs by 40%"
        ]
        return random.choice(alternatives)
    
    def calculate_fuel_consumption(self, distance, vehicle_type):
        """Calculate fuel consumption based on vehicle type"""
        mileage = {
            'car': 15,    # km per liter
            'bike': 50,   # km per liter
            'truck': 5    # km per liter
        }.get(vehicle_type, 15)
        
        fuel_consumed = distance / mileage
        return fuel_consumed
    
    def find_shortest_path(self):
        """Find the shortest path between two cities"""
        try:
            start_city = self.start_city_combo.currentText()
            end_city = self.end_city_combo.currentText()
            
            if not start_city or not end_city:
                QMessageBox.warning(self, "Error", "Please select both start and end cities")
                return
                
            if start_city == end_city:
                QMessageBox.warning(self, "Error", "Start and end cities are the same")
                return
                
            algorithm = self.algorithm_combo.currentData()
            vehicle_type = self.vehicle_combo.currentData()
            fuel_cost = self.fuel_cost_input.value()
            
            if algorithm == "compare":
                # Compare both algorithms
                dijkstra_path, dijkstra_distance, dijkstra_steps = self.graph.dijkstra_shortest_path(start_city, end_city)
                bfs_path, bfs_distance, bfs_steps = self.graph.bfs_shortest_path(start_city, end_city)
                
                if dijkstra_path and bfs_path:
                    self.last_path = dijkstra_path  # Use Dijkstra's path for visualization
                    
                    # Calculate fuel consumption for Dijkstra's path (real distance)
                    dijkstra_fuel = self.calculate_fuel_consumption(dijkstra_distance, vehicle_type)
                    dijkstra_total_cost = dijkstra_fuel * fuel_cost
                    
                    # Calculate fuel consumption for BFS path (step count)
                    bfs_fuel = self.calculate_fuel_consumption(bfs_distance, vehicle_type)
                    bfs_total_cost = bfs_fuel * fuel_cost
                    
                    # Show comparison results
                    comparison_text = (
                        f"<b>Algorithm Comparison:</b><br>"
                        f"<b>Dijkstra's:</b> {dijkstra_distance:.2f} km, {dijkstra_steps} steps, "
                        f"Fuel: {dijkstra_fuel:.2f}L (₹{dijkstra_total_cost:.2f})<br>"
                        f"<b>BFS:</b> {bfs_distance} steps, {bfs_distance} km (unweighted), "
                        f"Fuel: {bfs_fuel:.2f}L (₹{bfs_total_cost:.2f})<br>"
                        f"<b>Difference:</b> {abs(dijkstra_distance - bfs_distance):.2f} km, "
                        f"{abs(dijkstra_steps - bfs_steps)} steps"
                    )
                    self.comparison_result.setText(comparison_text)
                    self.comparison_result.show()
                    
                    # Show Dijkstra's path details (since it's more accurate for real distances)
                    self.path_result.setText(f"Optimal Path (Dijkstra's): {' → '.join(dijkstra_path)}")
                    self.distance_result.setText(f"Total distance: {dijkstra_distance:.2f} km")
                    self.fuel_result.setText(
                        f"Fuel needed: {dijkstra_fuel:.2f}L (Vehicle: {self.vehicle_combo.currentText()})"
                    )
                    self.fuel_cost_result.setText(f"Estimated fuel cost: ₹{dijkstra_total_cost:.2f}")
                    
                    # Update traffic display
                    path_coords = []
                    city_names = []
                    for city in dijkstra_path:
                        coords = self.graph.city_coords.get(city)
                        if coords and coords[0] is not None:
                            path_coords.append((coords[0], coords[1]))
                            city_names.append(city)
                    
                    traffic_factor = self.get_traffic_data(path_coords)
                    self.traffic_label.setText(f"🚦 Traffic: {'Heavy' if traffic_factor > 1 else 'Light'} "
                                             f"(×{traffic_factor} travel time)")
                    
                    # Update eco score
                    eco_score = self.calculate_eco_score(dijkstra_path)
                    self.eco_label.setText(f"🌱 Eco Score: {eco_score}/100")
                    
                    # Generate AI suggestion
                    self.ai_label.setText(f"🤖 AI Suggestion: {self.generate_ai_suggestion()}")
                    
                    # Update visualizations
                    self.graph_canvas.draw_graph(self.graph, dijkstra_path, start_city, end_city)
                    
                    # Show path on map if coordinates are available
                    if len(path_coords) >= 2:
                        self.map_view.show_route(path_coords, city_names)
                        self.tab_widget.setCurrentIndex(1)  # Switch to map view
                else:
                    self.comparison_result.hide()
                    self.path_result.setText("No path found between the selected cities")
                    self.distance_result.clear()
                    self.fuel_result.clear()
                    self.fuel_cost_result.clear()
                    self.traffic_label.setText("🚦 Traffic: Not calculated")
                    self.eco_label.setText("🌱 Eco Score: Not calculated")
                    self.ai_label.setText("🤖 AI Suggestion: Try a different route")
            else:
                # Single algorithm mode
                self.comparison_result.hide()
                
                if algorithm == "dijkstra":
                    path, distance, _ = self.graph.dijkstra_shortest_path(start_city, end_city)
                else:  # bfs
                    path, distance, _ = self.graph.bfs_shortest_path(start_city, end_city)
                
                if path:
                    self.last_path = path
                    
                    # Update results display
                    self.path_result.setText(f"Path: {' → '.join(path)}")
                    self.distance_result.setText(f"Total distance: {distance:.2f} km")
                    
                    # Calculate fuel consumption
                    fuel_consumed = self.calculate_fuel_consumption(distance, vehicle_type)
                    total_cost = fuel_consumed * fuel_cost
                    self.fuel_result.setText(
                        f"Fuel needed: {fuel_consumed:.2f}L (Vehicle: {self.vehicle_combo.currentText()})"
                    )
                    self.fuel_cost_result.setText(f"Estimated fuel cost: ₹{total_cost:.2f}")
                    
                    # Update traffic display
                    path_coords = []
                    city_names = []
                    for city in path:
                        coords = self.graph.city_coords.get(city)
                        if coords and coords[0] is not None:
                            path_coords.append((coords[0], coords[1]))
                            city_names.append(city)
                    
                    traffic_factor = self.get_traffic_data(path_coords)
                    self.traffic_label.setText(f"🚦 Traffic: {'Heavy' if traffic_factor > 1 else 'Light'} "
                                             f"(×{traffic_factor} travel time)")
                    
                    # Update eco score
                    eco_score = self.calculate_eco_score(path)
                    self.eco_label.setText(f"🌱 Eco Score: {eco_score}/100")
                    
                    # Generate AI suggestion
                    self.ai_label.setText(f"🤖 AI Suggestion: {self.generate_ai_suggestion()}")
                    
                    # Update visualizations
                    self.graph_canvas.draw_graph(self.graph, path, start_city, end_city)
                    
                    # Show path on map if coordinates are available
                    if len(path_coords) >= 2:
                        self.map_view.show_route(path_coords, city_names)
                        self.tab_widget.setCurrentIndex(1)  # Switch to map view
                else:
                    self.path_result.setText("No path found between the selected cities")
                    self.distance_result.clear()
                    self.fuel_result.clear()
                    self.fuel_cost_result.clear()
                    self.traffic_label.setText("🚦 Traffic: Not calculated")
                    self.eco_label.setText("🌱 Eco Score: Not calculated")
                    self.ai_label.setText("🤖 AI Suggestion: Try a different route")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")
    
    def enable_voice_navigation(self):
        """Read directions aloud"""
        if not self.last_path:
            QMessageBox.warning(self, "Error", "No route to announce. Please find a path first.")
            return
            
        try:
            engine = pyttsx3.init()
            directions = " then ".join(self.last_path)
            engine.say(f"Recommended route: {directions}")
            engine.runAndWait()
        except Exception as e:
            QMessageBox.warning(self, "Voice Error", f"Could not initialize voice: {str(e)}")
    
    def save_graph(self):
        """Save the current graph to a JSON file"""
        filename, _ = QFileDialog.getSaveFileName(self, "Save Graph", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.graph.to_dict(), f, indent=2)
                QMessageBox.information(self, "Success", "Graph saved successfully")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save graph: {str(e)}")
    
    def load_graph(self):
        """Load a graph from a JSON file"""
        filename, _ = QFileDialog.getOpenFileName(self, "Load Graph", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                self.graph = CityGraph.from_dict(data)
                self.update_city_lists()
                QMessageBox.information(self, "Success", "Graph loaded successfully")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load graph: {str(e)}")
    
    def load_sample_data(self):
        """Load sample data for Hyderabad locations with accurate coordinates"""
        sample_data = {
            'cities': {
                'Gachibowli': {'lat': 17.4401, 'lon': 78.3490},
                'Banjara Hills': {'lat': 17.4126, 'lon': 78.4440},
                'Secunderabad': {'lat': 17.4399, 'lon': 78.4983},
                'Hitech City': {'lat': 17.4474, 'lon': 78.3765},
                'Charminar': {'lat': 17.3616, 'lon': 78.4747},
                'Kukatpally': {'lat': 17.4849, 'lon': 78.4138},
                'Madhapur': {'lat': 17.4487, 'lon': 78.3907},
                'Jubilee Hills': {'lat': 17.4254, 'lon': 78.4085},
                'Abids': {'lat': 17.3926, 'lon': 78.4736},
                'Tolichowki': {'lat': 17.3595, 'lon': 78.4365}
            },
            'roads': {
                'Gachibowli': [('Hitech City', 5.2), ('Madhapur', 3.1)],
                'Banjara Hills': [('Jubilee Hills', 4.5), ('Abids', 6.8)],
                'Secunderabad': [('Abids', 8.2)],
                'Hitech City': [('Madhapur', 2.3), ('Kukatpally', 7.1)],
                'Charminar': [('Tolichowki', 3.7), ('Abids', 4.9)],
                'Kukatpally': [('Madhapur', 6.5)],
                'Madhapur': [('Jubilee Hills', 5.8)],
                'Jubilee Hills': [('Banjara Hills', 4.5)],
                'Abids': [('Tolichowki', 5.2)],
                'Tolichowki': []
            }
        }
        
        self.graph = CityGraph.from_dict(sample_data)
        self.update_city_lists()
        QMessageBox.information(self, "Sample Data", "Loaded sample data for Hyderabad locations with accurate distances")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern style
    
    window = RoutePlannerApp()
    window.show()
    sys.exit(app.exec_())