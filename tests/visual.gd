extends SceneTree

func _initialize() -> void:
	call_deferred("run")

func capture(filename: String) -> Image:
	for frame in range(12):
		await process_frame
	await RenderingServer.frame_post_draw
	var image := root.get_texture().get_image()
	var colors := {}
	for horizontal in range(0, image.get_width(), 24):
		for vertical in range(0, image.get_height(), 24):
			colors[image.get_pixel(horizontal, vertical).to_html()] = true
	assert(colors.size() > 30, "Rendered view is blank or missing geometry")
	assert(image.save_png("res://.cache/" + filename) == OK)
	print("CAPTURE %s: %dx%d, %d sampled colors" % [filename, image.get_width(), image.get_height(), colors.size()])
	return image

func capture_livery(game: Node3D) -> void:
	var bus = game.bus
	var camera: Camera3D = bus.camera
	game.city.visible = false
	game.hud.visible = false
	var views := {"bus_right.png": [Vector3(9, 2.2, -2.5), Vector3(0, 1.6, -0.5)], "bus_left.png": [Vector3(-9, 2.2, 1), Vector3(0, 1.6, 0)],
		"bus_front.png": [Vector3(3.5, 2.0, -13), Vector3(0, 1.5, -5)], "bus_rear.png": [Vector3(-3, 2.2, 12), Vector3(0, 1.6, 5)]}
	for filename in views:
		camera.global_position = bus.to_global(views[filename][0])
		camera.look_at(bus.to_global(views[filename][1]))
		await capture(filename)
	bus.doors_open = true
	bus.door_blend = 1.0
	bus.kneel = 1.0
	bus.animate(0.0)
	camera.global_position = bus.to_global(Vector3(7, 1.8, -3))
	camera.look_at(bus.to_global(Vector3(0, 1.2, -2)))
	await capture("bus_doors.png")
	bus.doors_open = false
	bus.door_blend = 0.0
	bus.kneel = 0.0
	bus.animate(0.0)
	bus.update_camera()
	game.city.visible = true
	game.hud.visible = true

func run() -> void:
	var game = load("res://scenes/main.tscn").instantiate()
	root.add_child(game)
	await process_frame
	for frame in range(60):
		await physics_frame
	game.bus.enabled = false
	var terrain = game.city.terrain
	var lidar = game.city.lidar
	assert(not lidar.metadata.is_empty(), "Bundled LiDAR must load")
	assert(lidar.measured_building_count == int(lidar.metadata.building_count), "All measured buildings must render")
	assert(lidar.tree_count == int(lidar.metadata.tree_count), "All imported canopy proxies must render")
	assert(lidar.measured_building_count > 100 and lidar.tree_count > 100, "LiDAR must provide substantial route scenery")
	var registry: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/landmarks_108.json"))
	assert(lidar.landmark_count == registry.landmarks.size(), "Every generated landmark must load exactly once")
	for landmark in registry.landmarks:
		var identifier := str(landmark.osm_id)
		assert(lidar.landmark_meshes.has(identifier), "Generated landmark mesh is missing: " + identifier)
		assert(lidar.buildings_by_id[identifier].roof_type == "native_lidar_tin", "Landmark must bypass coarse roof fitting")
		var mesh: MeshInstance3D = lidar.landmark_meshes[identifier]
		assert(mesh.mesh.get_surface_count() == 1, "Landmark must have an indexed detail mesh")
		assert(mesh.mesh.surface_get_array_len(0) == int(landmark.sampling.vertex_count), "Native landmark vertices must be preserved")
		assert(mesh.mesh.surface_get_array_index_len(0) == int(landmark.sampling.triangle_count) * 3, "Native landmark triangles must be preserved")
		assert(is_instance_valid(lidar.landmark_photo_materials[identifier].albedo_texture), "Native roof image must load for photo mode")
		assert(mesh.material_override == lidar.landmark_stylised_material, "Landmarks must start in stylised mode")
	var coarse_data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/lidar_108.json"))
	for building in coarse_data.buildings:
		if not lidar.landmark_meshes.has(str(building.osm_id)):
			assert(lidar.buildings_by_id[str(building.osm_id)] == building, "Unselected buildings must retain their original data")
	var instance_count := 0
	for batch in lidar.canopy_batches:
		instance_count += batch.multimesh.instance_count
	assert(instance_count == lidar.tree_count * 4, "Each canopy proxy needs a trunk and three crown lobes")
	assert(is_instance_valid(terrain.orthophoto_texture), "Bundled orthophoto must load")
	var shifted: Dictionary = terrain.orthophoto_metadata.duplicate()
	shifted.origin_lon = float(shifted.origin_lon) + 0.00001
	assert(not terrain.orthophoto_fits(shifted), "A shifted image origin must be rejected")
	shifted = terrain.orthophoto_metadata.duplicate()
	shifted.width_m = float(shifted.width_m) + 1.0
	assert(not terrain.orthophoto_fits(shifted), "A resized terrain crop requires new imagery")
	for chunk in terrain.get_children():
		if chunk is MeshInstance3D:
			var arrays: Array = chunk.mesh.surface_get_arrays(0)
			var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
			var texture_coordinates: PackedVector2Array = arrays[Mesh.ARRAY_TEX_UV]
			assert(vertices.size() == texture_coordinates.size(), "Every terrain vertex requires imagery coordinates")
			for index in [0, vertices.size() - 1]:
				var mapped := Vector2(terrain.grid_start.x, terrain.grid_start.y) + texture_coordinates[index] * Vector2((terrain.columns - 1) * terrain.spacing, (terrain.rows - 1) * terrain.spacing)
				assert(mapped.distance_to(Vector2(vertices[index].x, vertices[index].z)) < 0.002, "Imagery must align across terrain chunks")
	await capture("chase.png")
	game.bus.cab_view = true
	game.bus.update_camera()
	await capture("cab.png")
	game.bus.cab_view = false
	await capture_livery(game)
	game.next_stop = 5
	game.reset_bus()
	game.bus.enabled = true
	for frame in range(60):
		await physics_frame
	game.bus.enabled = false
	root.size = Vector2i(960, 540)
	var imagery := await capture("compact.png")
	for batch in lidar.canopy_batches:
		batch.visible = false
	var without_trees := await capture("lidar_without_trees.png")
	var vegetation_changes := 0
	for horizontal in range(0, imagery.get_width(), 8):
		for vertical in range(0, imagery.get_height(), 8):
			if imagery.get_pixel(horizontal, vertical) != without_trees.get_pixel(horizontal, vertical):
				vegetation_changes += 1
	assert(vegetation_changes > 100, "LiDAR vegetation must visibly change the route view")
	for batch in lidar.canopy_batches:
		batch.visible = true
	print("LIDAR: %d buildings, %d canopy proxies, %d changed pixel samples" % [lidar.measured_building_count, lidar.tree_count, vegetation_changes])
	var road = game.city.road
	assert(road.available and road.mesh_instances.size() == road.direction.chunks.size(), "Detailed road chunks must load")
	assert(not terrain.photo_mode and not road.photo_mode and not lidar.photo_mode, "Rendering must start stylised")
	assert(game.city.street_network.visible and game.city.street_network.get_child_count() > 0, "Stylised side streets must be drawn")
	var before: float = terrain.height_at(game.bus.position)
	var toggle := InputEventAction.new()
	toggle.action = "orthophoto"
	toggle.pressed = true
	game._unhandled_input(toggle)
	assert(terrain.photo_mode and road.photo_mode and lidar.photo_mode, "O must switch every layer to photo mode")
	assert(road.mesh_instances[0].mesh.surface_get_material(0) == road.photo_materials[0], "Roads must use imagery in photo mode")
	assert(not game.city.street_network.visible, "Stylised streets must hide over photographed streets")
	assert(terrain.height_at(game.bus.position) == before, "Mode toggle must not modify terrain heights")
	var photo := await capture("compact_photo.png")
	var changed := 0
	for horizontal in range(0, photo.get_width(), 8):
		for vertical in range(0, photo.get_height(), 8):
			if photo.get_pixel(horizontal, vertical) != imagery.get_pixel(horizontal, vertical):
				changed += 1
	assert(changed > 100, "Photo mode must visibly change the rendering")
	game._unhandled_input(toggle)
	assert(not terrain.photo_mode and road.mesh_instances[0].mesh.surface_get_material(0) == road.stylised_material, "O must restore stylised rendering")
	print("RENDER MODES: stylised default, photo toggle with %d changed pixel samples" % changed)
	assert(int(terrain.orthophoto_metadata.route_shape_samples_with_imagery) == int(terrain.orthophoto_metadata.route_shape_samples), "Bundled imagery must cover every route shape sample")
	assert(terrain.orthophoto_metadata.stops_without_imagery.is_empty(), "Bundled imagery must cover every stop")
	game.next_stop = game.city.route.stops.size() - 1
	game.reset_bus()
	game.bus.enabled = true
	for frame in range(60):
		await physics_frame
	game.bus.enabled = false
	game.bus.camera.global_position = game.bus.global_position + Vector3(0, 100, 40)
	game.bus.camera.look_at(game.bus.global_position)
	await capture("chelm.png")
	for child in game.get_children():
		if child is CanvasLayer:
			child.hide()
	var selections: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/landmark_selections.json"))
	for landmark in registry.landmarks:
		var identifier := str(landmark.osm_id)
		var key := str(landmark.get("selection_key", identifier))
		for selection in selections.landmarks:
			if str(selection.get("expected_osm_id", "")) == identifier:
				key = str(selection.key)
		var mesh: MeshInstance3D = lidar.landmark_meshes[identifier]
		var bounds := mesh.get_aabb()
		var center := mesh.to_global(bounds.get_center())
		var extent := maxf(bounds.size.x, bounds.size.z)
		root.size = Vector2i(960, 540)
		game.bus.camera.global_position = center + Vector3(1.05, 0.8, 1.0) * extent
		game.bus.camera.look_at(center)
		await capture(key + "_detail.png")
		root.size = Vector2i(1280, 720)
		game.bus.camera.global_position = center + Vector3(0.95, 0.15, 0.3) * extent
		game.bus.camera.look_at(center)
		await capture(key + "_street.png")
		print("LANDMARK %s: %d native vertices / %d triangles" % [key, landmark.sampling.vertex_count, landmark.sampling.triangle_count])
	print("VISUAL: PASS")
	game.queue_free()
	await process_frame
	quit()