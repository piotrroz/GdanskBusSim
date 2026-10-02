extends SceneTree

var failures := 0

func check(condition: bool, message: String) -> void:
	if not condition:
		push_error(message)
		failures += 1

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	var game = load("res://scenes/main.tscn").instantiate()
	root.add_child(game)
	await process_frame
	check(game.data.directions.size() == 2, "Two directions are required")
	if "--ground-only" in OS.get_cmdline_user_args():
		await check_ground_support(game)
		print("GROUND: %s" % ("PASS" if failures == 0 else "%d failures" % failures))
		game.queue_free()
		await process_frame
		quit(0 if failures == 0 else 1)
		return
	if "--terrain-only" in OS.get_cmdline_user_args():
		await check_terrain(game)
		print("TERRAIN: %s" % ("PASS" if failures == 0 else "%d failures" % failures))
		game.queue_free()
		await process_frame
		quit(0 if failures == 0 else 1)
		return
	if "--curb-only" in OS.get_cmdline_user_args():
		await check_curb(game)
		print("CURB: %s" % ("PASS" if failures == 0 else "%d failures" % failures))
		game.queue_free()
		await process_frame
		quit(0 if failures == 0 else 1)
		return
	for direction in range(2):
		game.load_direction(direction)
		await physics_frame
		check(not game.city.lidar.metadata.is_empty(), "LiDAR must align in both route directions")
		check(game.city.lidar.measured_building_count == int(game.city.lidar.metadata.get("building_count", -1)), "Measured building count must match imported data")
		check(game.city.lidar.tree_count == int(game.city.lidar.metadata.get("tree_count", -1)), "Vegetation count must match imported data")
		var registry: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/landmarks_108.json"))
		check(game.city.lidar.landmark_count == registry.landmarks.size(), "All generated landmarks must load exactly once")
		for landmark in registry.landmarks:
			check(game.city.lidar.landmark_meshes.has(str(landmark.osm_id)), "Landmark must load in both directions: " + str(landmark.osm_id))
		check(game.city.lidar.find_children("*", "CollisionObject3D", true, false).is_empty(), "LiDAR visual layer must not add driving obstacles")
		check(game.city.route.stops.size() == (13 if direction == 0 else 12), "Published stop count")
		check(game.city.points.size() > 300, "Detailed shape, not straight stop-to-stop lines")
		for index in range(1, game.city.stop_distances.size()):
			check(game.city.stop_distances[index] > game.city.stop_distances[index - 1], "Stop progress must be strictly increasing")
		check(game.bus.position.distance_to(game.city.stop_positions[0]) < 19, "Spawn must be serviceable")
		check(game.bus.wheels.size() == 4 and game.bus.door_panels.size() == 3, "Bus axles and three-door layout")
		game.bus.speed = 3
		game.bus.toggle_doors()
		check(not game.bus.doors_open, "Doors cannot open in motion")
		game.bus.speed = 0
		game.bus.toggle_doors()
		Input.action_press("accelerate")
		for frame in range(30):
			await physics_frame
		Input.action_release("accelerate")
		check(absf(game.bus.speed) < 0.01, "Open doors inhibit traction")
		game.update_service(5.1)
		check(game.served == 1 and game.next_stop == 1, "Origin boarding advances exactly one stop")
		game.update_service(6)
		check(game.served == 1, "Cannot board a distant stop")
		game.bus.doors_open = false
		Input.action_press("accelerate")
		for frame in range(60):
			await physics_frame
		Input.action_release("accelerate")
		check(game.bus.speed > 0.5, "Closed doors allow acceleration")
		game.reset_bus()
		check(game.bus.speed == 0 and not game.bus.doors_open, "Recovery resets motion and doors")
		for index in range(1, game.city.stop_positions.size()):
			game.reset_bus()
			game.bus.toggle_doors()
			game.update_service(5.1)
		check(game.finished and game.served == game.city.stop_positions.size(), "All stops complete the service")
		check(game.passengers == 0, "Terminus unloads all passengers")
	await check_ground_support(game)
	await check_terrain(game)
	print("SMOKE: %s" % ("PASS" if failures == 0 else "%d failures" % failures))
	game.queue_free()
	await process_frame
	quit(0 if failures == 0 else 1)

func building_bodies(game: Node3D) -> Array[RID]:
	var excluded: Array[RID] = [game.bus.get_rid()]
	for body in game.city.find_children("*", "StaticBody3D", true, false):
		if not game.city.terrain.is_ancestor_of(body) and not game.city.road.is_ancestor_of(body):
			excluded.append(body.get_rid())
	return excluded

func road_frame(road: Node3D, index: int) -> Dictionary:
	var centre: Vector3 = road.centerline[index]
	var ahead: Vector3 = road.centerline[mini(index + 1, road.centerline.size() - 1)]
	var behind: Vector3 = road.centerline[maxi(index - 1, 0)]
	var tangent := Vector3(ahead.x - behind.x, 0, ahead.z - behind.z).normalized()
	return {"centre": centre, "tangent": tangent, "right": tangent.cross(Vector3.UP)}

func surface_height(game: Node3D, location: Vector3, excluded: Array[RID]) -> float:
	var query := PhysicsRayQueryParameters3D.create(location + Vector3.UP * 2.5, location - Vector3.UP * 2.5, 3)
	query.exclude = excluded
	var hit := game.get_world_3d().direct_space_state.intersect_ray(query)
	return NAN if hit.is_empty() else float(hit.position.y)

func check_curb(game: Node3D) -> void:
	for direction in range(2):
		game.load_direction(direction)
		await physics_frame
		await physics_frame
		var road = game.city.road
		check(road.available, "Detailed road required for curb checks")
		var excluded := building_bodies(game)
		var worst := 0.0
		var worst_embankment := 0.0
		var profiles := 0
		var embankments := 0
		var structure_edges := 0
		# Offset 7 keeps samples off 512-section chunk seams, where rays can slip between collision shapes.
		for index in range(47, road.centerline.size() - 40, 40):
			var frame := road_frame(road, index)
			for side in [-1.0, 1.0]:
				var previous := NAN
				var heights: Array[float] = []
				var steepest := 0.0
				for step in range(11):
					# Offset avoids sampling exactly on the 4.5 m road/verge seam.
					var location: Vector3 = frame.centre + frame.right * side * (3.55 + step * 0.25)
					var height := surface_height(game, location, excluded)
					if not is_nan(height):
						heights.append(height)
					if not is_nan(previous) and not is_nan(height):
						steepest = maxf(steepest, absf(height - previous) / 0.25)
					previous = height
				if heights.is_empty():
					continue
				# Kerb-sized differences must be drivable ramps; real embankments only must not be vertical walls.
				if heights.max() - heights.min() <= 0.5:
					if steepest > worst and "--curb-debug" in OS.get_cmdline_user_args():
						print("worst candidate index %d side %d: %s" % [index, side, heights])
					worst = maxf(worst, steepest)
				else:
					# A drop of over 1.5 m between adjacent samples is the side of a bridge approach or deck, not a verge.
					var structure_edge := false
					for sample in range(1, heights.size()):
						structure_edge = structure_edge or absf(heights[sample] - heights[sample - 1]) > 1.5
					if structure_edge:
						structure_edges += 1
						continue
					if steepest > worst_embankment and "--curb-debug" in OS.get_cmdline_user_args():
						print("worst embankment index %d side %d at %s: %s" % [index, side, frame.centre, heights])
					worst_embankment = maxf(worst_embankment, steepest)
					embankments += 1
				profiles += 1
		print("Curb direction %d: %d edge profiles, steepest kerb-sized gradient %.2f; %d embankments, steepest %.2f; %d raised-structure edges" % [direction, profiles, worst, embankments, worst_embankment, structure_edges])
		check(worst < 1.0, "Kerb-sized road edges must stay climbable, below 45 degrees (gradient %.2f)" % worst)
		check(worst_embankment < 2.0, "Embankment edges must not form vertical walls (gradient %.2f)" % worst_embankment)
		var space := game.get_world_3d().direct_space_state
		var clearance := BoxShape3D.new()
		clearance.size = Vector3(9.0, 3.0, 24.0)
		var start_index := -1
		for index in range(400, road.centerline.size() - 400, 200):
			var frame := road_frame(road, index)
			var shape_query := PhysicsShapeQueryParameters3D.new()
			shape_query.shape = clearance
			shape_query.collision_mask = 1
			shape_query.exclude = [game.bus.get_rid()]
			var centre: Vector3 = frame.centre + frame.right * 6.0 + Vector3.UP * 2.2
			shape_query.transform = Transform3D(Basis(Vector3.UP, atan2(-frame.tangent.x, -frame.tangent.z)), centre)
			var blocked := false
			for hit in space.intersect_shape(shape_query, 8):
				if not game.city.terrain.is_ancestor_of(hit.collider):
					blocked = true
			if not blocked and absf(surface_height(game, frame.centre + frame.right * 8.0, excluded) - frame.centre.y) < 0.4:
				start_index = index
				break
		check(start_index >= 0, "No clear verge found for the drive-back test")
		if start_index < 0:
			continue
		var frame := road_frame(road, start_index)
		var heading: Vector3 = (frame.tangent * cos(0.45) - frame.right * sin(0.45)).normalized()
		game.bus.position = frame.centre + frame.right * 8.0
		game.bus.position.y = game.city.height_at(game.bus.position) + 0.7
		game.bus.rotation = Vector3(0, atan2(-heading.x, -heading.z), 0)
		game.bus.speed = 0
		game.bus.velocity = Vector3.ZERO
		for frame_index in range(30):
			await physics_frame
		Input.action_press("accelerate")
		var lateral := 8.0
		var entry_speed := 0.0
		for frame_index in range(600):
			await physics_frame
			lateral = (game.bus.position - frame.centre).dot(frame.right)
			if lateral < 2.5:
				entry_speed = game.bus.speed
				break
		Input.action_release("accelerate")
		var road_query := PhysicsRayQueryParameters3D.create(game.bus.position + Vector3.UP, game.bus.position - Vector3.UP, 2)
		var on_road := game.get_world_3d().direct_space_state.intersect_ray(road_query)
		print("Drive-back direction %d: lateral %.2f m, entry speed %.2f m/s" % [direction, lateral, entry_speed])
		check(lateral < 2.5 and entry_speed > 1.0, "Bus must drive from the verge back onto the road")
		check(not on_road.is_empty() and absf(on_road.position.y - game.bus.position.y) < 0.35, "Bus must end up supported by the road surface")

func check_ground_support(game: Node3D) -> void:
	for direction in range(2):
		game.load_direction(direction)
		await physics_frame
		for index in range(game.city.stop_positions.size()):
			game.next_stop = index
			game.reset_bus()
			for frame in range(60):
				await physics_frame
			var height: float = game.bus.position.y - game.city.height_at(game.bus.position)
			check(height > -0.15 and height < 1.5 and game.bus.is_on_floor(),
				"Ground clearance at %s: bus offset=%.3f, x=%.1f, z=%.1f" % [
					game.city.route.stops[index].name, height, game.bus.position.x, game.bus.position.z])

func check_terrain(game: Node3D) -> void:
	check(is_instance_valid(game.city.terrain), "Bundled terrain must be loaded")
	if not is_instance_valid(game.city.terrain):
		return
	for direction in range(2):
		game.load_direction(direction)
		await physics_frame
		await physics_frame
		var terrain = game.city.terrain
		check(float(terrain.metadata.height_max) - float(terrain.metadata.height_min) > 60, "NMT must contain the city hills")
		var sampled := 0
		for index in range(game.city.points.size() - 1):
			var start: Vector3 = game.city.points[index]
			var finish: Vector3 = game.city.points[index + 1]
			var tangent := Vector3(finish.x - start.x, 0, finish.z - start.z).normalized()
			var divisions := maxi(1, ceili(start.distance_to(finish) / 10.0))
			for section in range(divisions):
				var location := start.lerp(finish, float(section) / divisions) + tangent.cross(Vector3.UP) * 2.5
				check(terrain.contains(location), "Route must remain within imported coverage")
				location.y = game.city.height_at(location)
				var query := PhysicsRayQueryParameters3D.create(location + Vector3.UP * 0.4, location - Vector3.UP * 0.4, 1)
				query.exclude = [game.bus.get_rid()]
				var hit := game.get_world_3d().direct_space_state.intersect_ray(query)
				check(not hit.is_empty() and absf(hit.get("position", Vector3.INF).y - location.y) < 0.05,
					"Terrain collision mismatch at direction %d, segment %d" % [direction, index])
				sampled += 1
		print("Terrain direction %d: %d lane samples checked" % [direction, sampled])
		for stop_name in ["Cmentarna", "Worcella"]:
			for index in range(game.city.route.stops.size()):
				if str(game.city.route.stops[index].name).begins_with(stop_name):
					game.next_stop = index
					game.reset_bus()
					for frame in range(60):
						await physics_frame
					var before: Vector3 = game.bus.position
					Input.action_press("accelerate")
					for frame in range(180):
						await physics_frame
					Input.action_release("accelerate")
					check(before.distance_to(game.bus.position) > 2.0, "Bus must move on a slope at " + stop_name)
					var clearance: float = game.bus.position.y - game.city.height_at(game.bus.position)
					check(clearance > -0.15 and clearance < 1.5 and game.bus.is_on_floor(), "Bus must stay supported while driving at " + stop_name)