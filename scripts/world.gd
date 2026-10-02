extends Node3D

const Terrain = preload("res://scripts/terrain.gd")
const LidarScenery = preload("res://scripts/lidar_scenery.gd")
const RoadSurface = preload("res://scripts/road_network.gd")
var terrain: Node3D
var lidar: Node3D
var road: Node3D
const ROAD_LIFT := 0.12
var origin := Vector2.ZERO
var points := PackedVector3Array()
var distances := PackedFloat32Array()
var stop_positions := PackedVector3Array()
var stop_distances := PackedFloat32Array()
var route: Dictionary
var road_material: StandardMaterial3D
var stop_markers: Array[Node3D] = []
var street_network: Node3D
var route_grid: Dictionary = {}
var photo_mode := false
const ROUTE_GRID_CELL := 8.0

func set_photo_mode(enabled: bool) -> void:
	photo_mode = enabled
	if is_instance_valid(terrain):
		terrain.set_photo_mode(enabled)
	if is_instance_valid(road) and road.available:
		road.set_photo_mode(enabled)
	if is_instance_valid(lidar):
		lidar.set_photo_mode(enabled)
	# Photographed streets are already in the imagery; the stylised network would cover them.
	if is_instance_valid(street_network):
		street_network.visible = not enabled

func index_route_road() -> void:
	for index in range(0, road.centerline.size(), 4):
		var point: Vector3 = road.centerline[index]
		var key := Vector2i(floori(point.x / ROUTE_GRID_CELL), floori(point.z / ROUTE_GRID_CELL))
		if not route_grid.has(key):
			route_grid[key] = PackedVector2Array()
		route_grid[key].append(Vector2(point.x, point.z))

func near_route_road(location: Vector3, radius := 5.5) -> bool:
	var key := Vector2i(floori(location.x / ROUTE_GRID_CELL), floori(location.z / ROUTE_GRID_CELL))
	var target := Vector2(location.x, location.z)
	for dx in range(-1, 2):
		for dz in range(-1, 2):
			for point in route_grid.get(key + Vector2i(dx, dz), PackedVector2Array()):
				if point.distance_squared_to(target) < radius * radius:
					return true
	return false

func project(lon: float, lat: float) -> Vector3:
	var location := Vector3((lon - origin.x) * 111320.0 * cos(deg_to_rad(origin.y)), 0, -(lat - origin.y) * 111320.0)
	location.y = height_at(location)
	return location

func height_at(location: Vector3) -> float:
	return terrain.height_at(location) if is_instance_valid(terrain) else 0.0

# Route surface (bridges and tunnels included) near the bus route, terrain elsewhere.
func surface_height(location: Vector3) -> float:
	if is_instance_valid(road) and road.available:
		var height: float = road.route_height(location)
		if not is_nan(height):
			return height
	return height_at(location) + ROAD_LIFT

func paint(hex: String) -> StandardMaterial3D:
	var result := StandardMaterial3D.new()
	result.albedo_color = Color(hex)
	result.roughness = 0.93
	result.cull_mode = BaseMaterial3D.CULL_DISABLED
	return result

func box(size: Vector3, location: Vector3, color: Material, parent: Node3D = self) -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	instance.mesh = mesh
	instance.material_override = color
	parent.add_child(instance)
	instance.position = location
	return instance

func ribbon(path: PackedVector3Array, width: float, height: float, material: Material, inner_width := 0.0, parent: Node3D = null, avoid_route_road := false) -> void:
	if path.size() < 2:
		return
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	for index in range(path.size() - 1):
		var start := path[index]
		var finish := path[index + 1]
		var tangent := Vector3(finish.x - start.x, 0, finish.z - start.z).normalized()
		if tangent.length_squared() < 0.1:
			continue
		var side := tangent.cross(Vector3.UP)
		var bands := [Vector2(-width * 0.5, width * 0.5)]
		if inner_width > 0:
			bands = [Vector2(-width * 0.5, -inner_width * 0.5), Vector2(inner_width * 0.5, width * 0.5)]
		var divisions := maxi(1, ceili(start.distance_to(finish) / 2.5))
		for section in range(divisions):
			var near_point := start.lerp(finish, float(section) / divisions)
			var far_point := start.lerp(finish, float(section + 1) / divisions)
			if avoid_route_road and near_route_road((near_point + far_point) * 0.5):
				continue
			for band in bands:
				var lanes := maxi(1, ceili((band.y - band.x) / 2.0))
				for lane in range(lanes):
					var left := side * lerpf(band.x, band.y, float(lane) / lanes)
					var right := side * lerpf(band.x, band.y, float(lane + 1) / lanes)
					for vertex in [near_point + left, far_point + right, near_point + right, near_point + left, far_point + left, far_point + right]:
						vertex.y = height_at(vertex) + ROAD_LIFT + height
						surface.set_normal(Vector3.UP)
						surface.add_vertex(vertex)
	var instance := MeshInstance3D.new()
	instance.mesh = surface.commit()
	instance.material_override = material
	(parent if parent else self).add_child(instance)

func nearest(location: Vector3, minimum := 0.0) -> Dictionary:
	var best := INF
	var result := {"point": points[0], "distance": 0.0, "tangent": Vector3.FORWARD, "error": 0.0}
	for index in range(points.size() - 1):
		if distances[index + 1] < minimum:
			continue
		var segment := points[index + 1] - points[index]
		segment.y = 0
		var offset := location - points[index]
		offset.y = 0
		var fraction := clampf(offset.dot(segment) / maxf(segment.length_squared(), 0.001), 0, 1)
		var candidate := points[index] + segment * fraction
		candidate.y = surface_height(candidate)
		var error := Vector2(candidate.x - location.x, candidate.z - location.z).length_squared()
		if error < best:
			best = error
			result = {"point": candidate, "distance": distances[index] + segment.length() * fraction, "tangent": segment.normalized(), "error": sqrt(error)}
	return result

func build(direction: Dictionary) -> void:
	route = direction
	var first: Array = direction.points[0]
	origin = Vector2(float(first[0]), float(first[1]))
	if FileAccess.file_exists("res://data/terrain_108.json"):
		terrain = Terrain.new()
		add_child(terrain)
		terrain.load_data()
		origin = Vector2(float(terrain.metadata.origin_lon), float(terrain.metadata.origin_lat))
		road = RoadSurface.new()
		road.load_data(str(direction.shape_id), terrain.metadata)
		if road.available:
			road.carved_vertices = terrain.carve_below(road.surface_points(), 0.12)
		terrain.build()
		add_child(road)
		road.build(terrain)
		if road.available:
			index_route_road()
	street_network = Node3D.new()
	street_network.name = "StreetNetwork"
	add_child(street_network)
	for coordinate in direction.points:
		var location := project(float(coordinate[0]), float(coordinate[1]))
		if points.is_empty() or location.distance_to(points[-1]) > 0.15:
			points.append(location)
	distances.append(0)
	for index in range(1, points.size()):
		distances.append(distances[-1] + Vector2(points[index].x - points[index - 1].x, points[index].z - points[index - 1].z).length())
	road_material = paint("56585a")
	var fallback_height := -20.0 if is_instance_valid(terrain) else 0.0
	box(Vector3(12000, 0.3, 12000), Vector3(0, fallback_height - 0.2, 0), paint("829a70"))
	var ground := StaticBody3D.new()
	var ground_shape := CollisionShape3D.new()
	var ground_box := BoxShape3D.new()
	ground_box.size = Vector3(12000, 2, 12000)
	ground_shape.shape = ground_box
	ground_shape.position.y = fallback_height - 1.0
	ground.add_child(ground_shape)
	add_child(ground)
	load_osm()
	if not is_instance_valid(road) or not road.available:
		ribbon(points, 13.5, 0.006, paint("b5b7aa"), 9.0)
		ribbon(points, 9.0, 0.018, road_material)
	var line_material := paint("e6e4ca")
	for index in range(1, points.size()) if not is_instance_valid(road) or not road.available else []:
		var segment := points[index] - points[index - 1]
		var tangent := segment.normalized()
		var length := segment.length()
		var cursor := 0.0
		while cursor + 3.0 < length:
			var center := points[index - 1] + tangent * (cursor + 1.5)
			center.y = height_at(center) + ROAD_LIFT + 0.05
			var marking := box(Vector3(0.12, 0.025, 3.0), center, line_material)
			marking.rotation.y = atan2(tangent.x, tangent.z)
			marking.rotation.x = -asin(tangent.y)
			cursor += 10.0
	var previous := 0.0
	for stop in route.stops:
		var actual := project(float(stop.lon), float(stop.lat))
		var match_point := nearest(actual, previous)
		previous = float(match_point.distance)
		stop_distances.append(previous)
		var right: Vector3 = match_point.tangent.cross(Vector3.UP)
		var platform: Vector3 = match_point.point + right * 6.4
		platform.y = maxf(height_at(platform) + ROAD_LIFT, match_point.point.y)
		stop_positions.append(platform)
		build_stop(platform, match_point.tangent, str(stop.name))
	build_guidance()

func build_guidance() -> void:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	var next_mark := stop_distances[0] + 20.0
	for index in range(points.size() - 1):
		var tangent := (points[index + 1] - points[index]).normalized()
		tangent.y = 0
		tangent = tangent.normalized()
		var right := tangent.cross(Vector3.UP)
		while next_mark < distances[index + 1] and next_mark < stop_distances[-1]:
			if next_mark >= distances[index]:
				var center := points[index] + tangent * (next_mark - distances[index]) + right * 2.5 + Vector3.UP * 0.065
				for vertex in [center + tangent * 1.4, center - tangent - right * 0.5, center - tangent + right * 0.5]:
					vertex.y = surface_height(vertex) + 0.1
					surface.set_normal(Vector3.UP)
					surface.add_vertex(vertex)
			next_mark += 28.0
	var instance := MeshInstance3D.new()
	instance.mesh = surface.commit()
	instance.material_override = paint("9cd9bd")
	add_child(instance)

func build_stop(location: Vector3, tangent: Vector3, title: String) -> void:
	var stop := Node3D.new()
	add_child(stop)
	stop.position = location
	stop.rotation.y = atan2(-tangent.x, -tangent.z)
	var concrete := paint("c3c5b8")
	var steel := paint("4d5b58")
	box(Vector3(3.4, 0.14, 16), Vector3(0.3, 0.07, 0), concrete, stop)
	box(Vector3(0.18, 0.03, 14), Vector3(-1.3, 0.16, 0), paint("e6cc64"), stop)
	box(Vector3(0.10, 3.5, 0.10), Vector3(-0.8, 1.75, -4.5), steel, stop)
	box(Vector3(0.70, 0.88, 0.08), Vector3(-0.8, 3.3, -4.5), paint("176397"), stop)
	box(Vector3(0.55, 0.57, 0.09), Vector3(-0.8, 3.32, -4.5), paint("f0f1e7"), stop)
	box(Vector3(0.36, 0.24, 0.10), Vector3(-0.8, 3.31, -4.5), steel, stop)
	box(Vector3(2.8, 0.13, 5.3), Vector3(0.6, 2.7, 1), paint("a82735"), stop)
	for longitudinal in [-1.45, 3.45]:
		box(Vector3(0.10, 2.65, 0.10), Vector3(1.7, 1.35, longitudinal), steel, stop)
	box(Vector3(0.06, 1.7, 4.8), Vector3(1.7, 1.6, 1), paint("91aaa5"), stop)
	box(Vector3(0.65, 0.13, 3.8), Vector3(0.95, 0.6, 1), paint("847969"), stop)
	var label := Label3D.new()
	label.text = title
	label.font_size = 36
	label.pixel_size = 0.016
	label.position = Vector3(0, 4.6, 0)
	label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	label.no_depth_test = false
	label.modulate = Color("fff8da")
	stop.add_child(label)
	stop_markers.append(label)

func load_osm() -> void:
	if not FileAccess.file_exists("res://data/map_108.json"):
		return
	lidar = LidarScenery.new()
	add_child(lidar)
	lidar.load_data(self)
	var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/map_108.json"))
	var buildings := SurfaceTool.new()
	buildings.begin(Mesh.PRIMITIVE_TRIANGLES)
	buildings.set_smooth_group(-1)
	var fallback := SurfaceTool.new()
	fallback.begin(Mesh.PRIMITIVE_TRIANGLES)
	fallback.set_smooth_group(-1)
	var fallback_count := 0
	var building_count := 0
	var building_material := paint("ffffff")
	building_material.vertex_color_use_as_albedo = true
	for element in data.elements:
		var tags: Dictionary = element.get("tags", {})
		var path := PackedVector3Array()
		for coordinate in element.get("geometry", []):
			path.append(project(float(coordinate.lon), float(coordinate.lat)))
		if path.size() < 2:
			continue
		if tags.has("highway"):
			if is_instance_valid(road) and road.available and road.included_way_ids.has(str(int(element.id))):
				continue
			var width := 6.5
			if str(tags.highway) in ["primary", "secondary", "trunk"]:
				width = 10.0
			if str(tags.highway) == "service":
				width = 4.0
			if is_instance_valid(road) and road.available:
				ribbon(path, width, 0.0, road_material, 0.0, street_network, true)
			else:
				ribbon(path, width, 0.0, road_material)
		elif tags.has("building") and path.size() >= 4:
			var identifier := str(int(element.id))
			var clipped: bool = is_instance_valid(road) and road.building_clips.has(identifier)
			var measured: bool = lidar.buildings_by_id.has(identifier) and not clipped
			var footprints: Array = []
			if clipped:
				# Footprint parts that do not overlap a bridge deck (e.g. a car park under a viaduct).
				for ring in road.building_clips[identifier]:
					var part := PackedVector3Array()
					for point in ring:
						part.append(Vector3(float(point[0]), height_at(Vector3(float(point[0]), 0, float(point[1]))), float(point[1])))
					part.append(part[0])
					footprints.append(part)
			else:
				footprints.append(path)
			for footprint in footprints:
				if add_building(buildings, fallback, footprint, tags, int(element.id), measured):
					building_count += 1
					if not measured:
						fallback_count += 1
	if building_count > 0:
		buildings.generate_normals()
		var instance := MeshInstance3D.new()
		instance.mesh = buildings.commit()
		instance.material_override = building_material
		add_child(instance)
		instance.create_trimesh_collision()
		instance.visible = false
	if fallback_count > 0:
		fallback.generate_normals()
		var instance := MeshInstance3D.new()
		instance.mesh = fallback.commit()
		instance.material_override = building_material
		add_child(instance)
	lidar.build()

func add_building(buildings: SurfaceTool, fallback: SurfaceTool, path: PackedVector3Array, tags: Dictionary, identifier: int, measured: bool) -> bool:
	var base_height := path[0].y
	for vertex in path:
		base_height = maxf(base_height, vertex.y)
	var polygon := PackedVector2Array()
	for index in range(path.size() - 1):
		polygon.append(Vector2(path[index].x, path[index].z))
	var indices := Geometry2D.triangulate_polygon(polygon)
	if indices.is_empty():
		return false
	var levels := float(str(tags.get("building:levels", "3")))
	var height := clampf(float(str(tags.get("height", str(levels * 3.2))).trim_suffix(" m")), 3.0, 65.0)
	var palette := [Color("c5b6a7"), Color("d5d2c7"), Color("a6aaa6"), Color("bd9b89"), Color("d9d9cd")]
	var color: Color = palette[identifier % palette.size()]
	for index in range(path.size() - 1):
		var start := Vector3(path[index].x, base_height - 4.0, path[index].z)
		var finish := Vector3(path[index + 1].x, base_height - 4.0, path[index + 1].z)
		var top := Vector3.UP * (height + 4.0)
		for vertex in [start, finish, finish + top, start, finish + top, start + top]:
			buildings.set_color(color)
			buildings.add_vertex(vertex)
			if not measured:
				fallback.set_color(color)
				fallback.add_vertex(vertex)
	for index in indices:
		buildings.set_color(color.darkened(0.24))
		buildings.add_vertex(Vector3(polygon[index].x, base_height + height, polygon[index].y))
		if not measured:
			fallback.set_color(color.darkened(0.24))
			fallback.add_vertex(Vector3(polygon[index].x, base_height + height, polygon[index].y))
	return true