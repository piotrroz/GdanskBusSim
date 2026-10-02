extends Node3D

var metadata: Dictionary
var buildings_by_id: Dictionary = {}
var measured_building_count := 0
var tree_count := 0
var canopy_batches: Array[MultiMeshInstance3D] = []
var landmark_meshes: Dictionary = {}
var landmark_photo_materials: Dictionary = {}
var landmark_count := 0
var city: Node3D
var roof_instance: MeshInstance3D
var roof_photo_material: StandardMaterial3D
var roof_stylised_material: StandardMaterial3D
var landmark_stylised_material: StandardMaterial3D
var photo_mode := false
const ROOF_PALETTE := [Color("9a5544"), Color("6c6f71"), Color("85624f"), Color("8e918d"), Color("a8664f")]

func load_data(owner_city: Node3D) -> void:
	city = owner_city
	if not FileAccess.file_exists("res://data/lidar_108.json") or not is_instance_valid(city.terrain):
		return
	var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/lidar_108.json"))
	var terrain: Dictionary = city.terrain.metadata
	if absf(float(data.origin_lon) - float(terrain.origin_lon)) > 0.000000001 or absf(float(data.origin_lat) - float(terrain.origin_lat)) > 0.000000001:
		push_warning("LiDAR scenery origin is stale; rerun tools/import_lidar.py.")
		return
	for key in data.terrain_grid:
		if data.terrain_grid[key] != terrain[key]:
			push_warning("LiDAR scenery terrain reference is stale; rerun tools/import_lidar.py.")
			return
	metadata = data
	load_landmarks(terrain)
	for building in data.buildings:
		buildings_by_id[str(building.osm_id)] = building

func load_landmarks(terrain: Dictionary) -> void:
	if not FileAccess.file_exists("res://data/landmarks_108.json"):
		return
	var registry: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/landmarks_108.json"))
	var map_data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/map_108.json"))
	var mapped_buildings: Dictionary = {}
	for element in map_data.elements:
		if element.get("tags", {}).has("building"):
			mapped_buildings[str(int(element.id))] = true
	var applied: Dictionary = {}
	for landmark in registry.landmarks:
		var identifier := str(landmark.osm_id)
		if applied.has(identifier) or not mapped_buildings.has(identifier):
			push_warning("Duplicate or unmapped landmark %s skipped." % identifier)
			continue
		var aligned := absf(float(landmark.origin_lon) - float(terrain.origin_lon)) < 0.000000001 and absf(float(landmark.origin_lat) - float(terrain.origin_lat)) < 0.000000001
		for key in landmark.terrain_grid:
			aligned = aligned and landmark.terrain_grid[key] == terrain[key]
		if not aligned:
			push_warning("Landmark %s terrain reference is stale; retaining coarse building." % landmark.osm_id)
			continue
		var found := false
		for index in range(metadata.buildings.size()):
			if str(metadata.buildings[index].osm_id) == str(landmark.osm_id):
				metadata.buildings[index] = landmark
				found = true
				break
		if not found:
			metadata.buildings.append(landmark)
		applied[identifier] = true
	metadata.building_count = metadata.buildings.size()

func point(value: Array) -> Vector3:
	return Vector3(float(value[0]), float(value[1]), float(value[2]))

func set_photo_mode(enabled: bool) -> void:
	photo_mode = enabled
	if is_instance_valid(roof_instance):
		roof_instance.material_override = roof_photo_material if photo_mode else roof_stylised_material
	for identifier in landmark_meshes:
		landmark_meshes[identifier].material_override = landmark_photo_materials[identifier] if photo_mode else landmark_stylised_material

func build() -> void:
	if metadata.is_empty():
		return
	landmark_stylised_material = StandardMaterial3D.new()
	landmark_stylised_material.albedo_color = Color("7f5446")
	landmark_stylised_material.roughness = 0.9
	landmark_stylised_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	build_buildings()
	build_trees()

func build_buildings() -> void:
	var walls := SurfaceTool.new()
	walls.begin(Mesh.PRIMITIVE_TRIANGLES)
	walls.set_smooth_group(-1)
	var roofs := SurfaceTool.new()
	roofs.begin(Mesh.PRIMITIVE_TRIANGLES)
	roofs.set_smooth_group(-1)
	var palette := [Color("d5c5b5"), Color("dadcd6"), Color("bcbfb7"), Color("d3b49e"), Color("e4ddcf")]
	var terrain = city.terrain
	for building in metadata.buildings:
		if is_instance_valid(city.road) and city.road.building_clips.has(str(building.osm_id)):
			continue
		var color: Color = palette[int(building.osm_id) % palette.size()]
		var roof_color: Color = ROOF_PALETTE[absi(hash(str(building.osm_id))) % ROOF_PALETTE.size()]
		var base := float(building.base_height)
		var outline: Array = building.wall_top
		var native_roof: bool = building.has("roof_vertices")
		var wall_distance := 0.0
		for index in range(outline.size()):
			var start := point(outline[index])
			var finish := point(outline[(index + 1) % outline.size()])
			var lower_start := Vector3(start.x, base - 4.0, start.z)
			var lower_finish := Vector3(finish.x, base - 4.0, finish.z)
			var length := Vector2(finish.x - start.x, finish.z - start.z).length()
			var start_distance := wall_distance if native_roof else 0.0
			var end_distance := start_distance + length
			var vertices := [lower_start, lower_finish, finish, lower_start, finish, start]
			var coordinates := [Vector2(start_distance, -4), Vector2(end_distance, -4), Vector2(end_distance, finish.y - base), Vector2(start_distance, -4), Vector2(end_distance, finish.y - base), Vector2(start_distance, start.y - base)]
			for vertex_index in range(6):
				walls.set_color(color)
				walls.set_uv(coordinates[vertex_index])
				walls.add_vertex(vertices[vertex_index])
			wall_distance += length
		if native_roof:
			build_landmark_roof(building)
		for triangle in building.get("roof_triangles", []):
			var vertices := [point(triangle[0]), point(triangle[1]), point(triangle[2])]
			if (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).y > 0:
				vertices.reverse()
			for vertex in vertices:
				roofs.set_color(roof_color)
				roofs.set_uv(Vector2((vertex.x - terrain.grid_start.x) / ((terrain.columns - 1) * terrain.spacing), (vertex.z - terrain.grid_start.y) / ((terrain.rows - 1) * terrain.spacing)))
				roofs.add_vertex(vertex)
		measured_building_count += 1
	var wall_shader := Shader.new()
	wall_shader.code = """
shader_type spatial;
render_mode cull_disabled;
void fragment() {
	vec2 cell = fract(UV / vec2(3.0, 3.2));
	float opening = step(0.22, cell.x) * step(cell.x, 0.76) * step(0.28, cell.y) * step(cell.y, 0.76) * step(1.0, UV.y);
	float frame = step(0.18, cell.x) * step(cell.x, 0.80) * step(0.24, cell.y) * step(cell.y, 0.80) * step(1.0, UV.y);
	float mullion = 1.0 - smoothstep(0.012, 0.025, abs(cell.x - 0.49));
	float sill = step(0.21, cell.y) * step(cell.y, 0.25) * frame;
	vec3 plaster = COLOR.rgb * (0.94 + 0.06 * step(0.04, fract(UV.y / 3.2)));
	vec3 window_color = mix(vec3(0.17, 0.25, 0.28), vec3(0.32, 0.40, 0.42), cell.y);
	ALBEDO = mix(plaster, vec3(0.76, 0.77, 0.70), frame);
	ALBEDO = mix(ALBEDO, window_color, opening * (1.0 - mullion));
	ALBEDO = mix(ALBEDO, vec3(0.83, 0.81, 0.73), sill);
	ROUGHNESS = mix(0.92, 0.32, opening);
}
"""
	var wall_material := ShaderMaterial.new()
	wall_material.shader = wall_shader
	finish_surface(walls, wall_material)
	var roof_material := StandardMaterial3D.new()
	roof_material.albedo_color = Color.WHITE if is_instance_valid(terrain.orthophoto_texture) else Color("796e65")
	roof_material.albedo_texture = terrain.orthophoto_texture
	roof_material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	roof_material.texture_repeat = false
	roof_material.roughness = 0.95
	roof_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	roof_photo_material = roof_material
	roof_stylised_material = StandardMaterial3D.new()
	roof_stylised_material.vertex_color_use_as_albedo = true
	roof_stylised_material.roughness = 0.9
	roof_stylised_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	roof_instance = finish_surface(roofs, roof_photo_material if photo_mode else roof_stylised_material)

func build_landmark_roof(building: Dictionary) -> void:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	var texture_info: Dictionary = building.roof_texture
	for value in building.roof_vertices:
		var vertex := point(value)
		surface.set_uv(Vector2((vertex.x - float(texture_info.grid_x)) / float(texture_info.width_m), (vertex.z - float(texture_info.grid_z)) / float(texture_info.height_m)))
		surface.add_vertex(vertex)
	var indices: Array = building.roof_indices
	for index in range(0, indices.size(), 3):
		var first := int(indices[index])
		var second := int(indices[index + 1])
		var third := int(indices[index + 2])
		var start := point(building.roof_vertices[first])
		var middle := point(building.roof_vertices[second])
		var finish := point(building.roof_vertices[third])
		surface.add_index(first)
		if (middle - start).cross(finish - start).y > 0:
			surface.add_index(third)
			surface.add_index(second)
		else:
			surface.add_index(second)
			surface.add_index(third)
	surface.generate_normals()
	var material := StandardMaterial3D.new()
	material.roughness = 0.95
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	material.texture_repeat = false
	var image := Image.new()
	var error := image.load_png_from_buffer(FileAccess.get_file_as_bytes(str(texture_info.file)))
	if error == OK:
		image.generate_mipmaps()
		material.albedo_texture = ImageTexture.create_from_image(image)
	else:
		material.albedo_color = Color("796e65")
		push_warning("Native landmark texture missing: %s" % texture_info.file)
	var instance := MeshInstance3D.new()
	instance.mesh = surface.commit()
	landmark_photo_materials[str(building.osm_id)] = material
	instance.material_override = material if photo_mode else landmark_stylised_material
	instance.name = "Landmark_" + str(building.osm_id)
	add_child(instance)
	landmark_meshes[str(building.osm_id)] = instance
	landmark_count += 1

func finish_surface(surface: SurfaceTool, material: Material) -> MeshInstance3D:
	surface.generate_normals()
	var instance := MeshInstance3D.new()
	instance.mesh = surface.commit()
	instance.material_override = material
	add_child(instance)
	return instance

func build_trees() -> void:
	var sphere := SphereMesh.new()
	sphere.radius = 1.0
	sphere.height = 2.0
	sphere.radial_segments = 16
	sphere.rings = 10
	var arrays := sphere.surface_get_arrays(0)
	var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
	var noise := FastNoiseLite.new()
	noise.seed = 108
	noise.frequency = 3.0
	for index in range(vertices.size()):
		vertices[index] *= 0.94 + noise.get_noise_3dv(vertices[index]) * 0.22
	arrays[Mesh.ARRAY_VERTEX] = vertices
	var crown := ArrayMesh.new()
	crown.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	var crown_material := StandardMaterial3D.new()
	crown_material.albedo_color = Color.WHITE
	crown_material.vertex_color_use_as_albedo = true
	crown_material.roughness = 1.0
	var trunk := CylinderMesh.new()
	trunk.top_radius = 0.55
	trunk.bottom_radius = 1.0
	trunk.height = 1.0
	trunk.radial_segments = 7
	var trunk_material := StandardMaterial3D.new()
	trunk_material.albedo_color = Color("5b5749")
	trunk_material.roughness = 1.0
	var groups: Dictionary = {}
	for tree in metadata.trees:
		var location := point(tree.position)
		var key := Vector2i(floori(location.x / 160), floori(location.z / 160))
		if not groups.has(key):
			groups[key] = []
		groups[key].append(tree)
	var palette := [Color("607545"), Color("72804b"), Color("516d47"), Color("8c9151"), Color("496b50")]
	for key in groups:
		var trees: Array = groups[key]
		var crowns := MultiMesh.new()
		crowns.transform_format = MultiMesh.TRANSFORM_3D
		crowns.use_colors = true
		crowns.mesh = crown
		crowns.instance_count = trees.size() * 3
		var trunks := MultiMesh.new()
		trunks.transform_format = MultiMesh.TRANSFORM_3D
		trunks.mesh = trunk
		trunks.instance_count = trees.size()
		for index in range(trees.size()):
			var tree: Dictionary = trees[index]
			var location := point(tree.position)
			var height := float(tree.height)
			var radius := float(tree.radius)
			var seed_value := absi(int(location.x * 31 + location.z * 17))
			var trunk_height := height * 0.62
			var trunk_radius := clampf(height * 0.018, 0.10, 0.42)
			trunks.set_instance_transform(index, Transform3D(Basis.IDENTITY.scaled(Vector3(trunk_radius, trunk_height, trunk_radius)), location + Vector3.UP * trunk_height * 0.5))
			for lobe in range(3):
				var angle := float(seed_value % 628) / 100.0 + lobe * TAU / 3
				var offset := Vector3(cos(angle), 0, sin(angle)) * radius * 0.28
				var half_height := height * (0.30 if lobe == 0 else 0.24)
				var position := location + offset + Vector3.UP * (height - half_height - lobe * height * 0.045)
				var basis := Basis(Vector3.UP, angle).scaled(Vector3(radius * 0.78, half_height, radius * 0.86))
				crowns.set_instance_transform(index * 3 + lobe, Transform3D(basis, position))
				crowns.set_instance_color(index * 3 + lobe, palette[(seed_value + lobe) % palette.size()])
			tree_count += 1
		add_batch(crowns, crown_material)
		add_batch(trunks, trunk_material)

func add_batch(multimesh: MultiMesh, material: Material) -> void:
	var instance := MultiMeshInstance3D.new()
	instance.multimesh = multimesh
	instance.material_override = material
	instance.visibility_range_end = 900.0
	add_child(instance)
	canopy_batches.append(instance)