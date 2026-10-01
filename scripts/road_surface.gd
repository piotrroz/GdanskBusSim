extends Node3D

var metadata: Dictionary
var direction: Dictionary
var available := false
var chunk_count := 0
var vertex_count := 0
var triangle_count := 0
var centerline := PackedVector3Array()
var mesh_instances: Array[MeshInstance3D] = []
var collision_bodies: Array[StaticBody3D] = []
var terrain: Node3D
var verge_material: StandardMaterial3D
var chunk_values: Array[PackedFloat32Array] = []
var carved_vertices := 0
var photo_materials: Array[StandardMaterial3D] = []
var stylised_material: ShaderMaterial
var photo_mode := false

const STYLISED_SHADER := """
shader_type spatial;
uniform vec3 asphalt : source_color = vec3(0.30, 0.31, 0.32);
uniform vec3 marking : source_color = vec3(0.93, 0.93, 0.89);
uniform float edge_line = 4.15;
varying vec3 world_position;
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float value_noise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x), mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
}
float band(float value, float centre, float half_width, float blur) {
	return 1.0 - smoothstep(half_width, half_width + blur, abs(value - centre));
}
void vertex() { world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }
void fragment() {
	// UV2: x = metres from route centre line, y = metres along the route.
	float x = UV2.x;
	float along = UV2.y;
	vec2 p = world_position.xz;
	float blur = fwidth(x) + 0.01;
	vec3 colour = asphalt * (0.9 + 0.12 * value_noise(p * 0.35) + 0.08 * value_noise(p * 3.1));
	float tracks = 0.0;
	for (int lane = -1; lane <= 1; lane += 2) {
		for (int wheel = -1; wheel <= 1; wheel += 2) {
			tracks = max(tracks, band(x, float(lane) * 2.1 + float(wheel) * 0.95, 0.22, 0.25));
		}
	}
	colour *= 1.0 - 0.07 * tracks;
	float edges = band(abs(x), edge_line, 0.06, blur);
	float dashes = band(x, 0.0, 0.06, blur) * step(fract(along / 12.0), 0.333);
	float paint = max(edges, dashes) * (0.8 + 0.2 * value_noise(p * 6.0));
	ALBEDO = mix(colour, marking, paint);
	ROUGHNESS = mix(0.92, 0.7, paint);
}
"""
const VERGE_MIN_WIDTH := 1.5
const VERGE_MAX_WIDTH := 4.0
const VERGE_TARGET_GRADE := 0.25
# Verge ends below the terrain surface so the two meshes cross without a ledge.
const VERGE_BURY := 0.06

func load_data(shape_id: String, terrain_metadata: Dictionary) -> void:
	if not FileAccess.file_exists("res://data/roads_108/roads_108.json"):
		return
	var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/roads_108/roads_108.json"))
	var aligned := absf(float(data.origin_lon) - float(terrain_metadata.origin_lon)) < 0.000000001 and absf(float(data.origin_lat) - float(terrain_metadata.origin_lat)) < 0.000000001
	for key in data.terrain_grid:
		aligned = aligned and data.terrain_grid[key] == terrain_metadata[key]
	if not aligned:
		push_warning("Detailed roads do not match terrain; retaining legacy roads.")
		return
	for candidate in data.directions:
		if str(candidate.shape_id) == shape_id:
			metadata = data
			direction = candidate
			available = true
			for chunk in direction.chunks:
				var values := FileAccess.get_file_as_bytes(str(chunk.position_file)).to_float32_array()
				assert(values.size() == int(chunk.sections) * int(chunk.columns) * int(chunk.stride), "Detailed-road binary dimensions mismatch")
				chunk_values.append(values)
			return
	push_warning("No detailed road for shape %s; retaining legacy roads." % shape_id)

func surface_points() -> PackedVector3Array:
	var points := PackedVector3Array()
	for index in range(chunk_values.size()):
		var values := chunk_values[index]
		var stride := int(direction.chunks[index].stride)
		for offset in range(0, values.size(), stride):
			points.append(Vector3(values[offset], values[offset + 1], values[offset + 2]))
	return points

func build(ground: Node3D) -> void:
	if not available:
		return
	terrain = ground
	var shader := Shader.new()
	shader.code = STYLISED_SHADER
	stylised_material = ShaderMaterial.new()
	stylised_material.shader = shader
	verge_material = StandardMaterial3D.new()
	verge_material.albedo_color = Color("a9a99c")
	verge_material.roughness = 1.0
	verge_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	for index in range(direction.chunks.size()):
		build_chunk(direction.chunks[index], chunk_values[index], index == 0, index == direction.chunks.size() - 1)
	assert(vertex_count == int(direction.vertex_count), "Detailed-road vertex count mismatch")
	assert(triangle_count == int(direction.triangle_count), "Detailed-road triangle count mismatch")

func build_chunk(chunk: Dictionary, values: PackedFloat32Array, first_chunk: bool, last_chunk: bool) -> void:
	var columns := int(chunk.columns)
	var sections := int(chunk.sections)
	var stride := int(chunk.stride)
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var texture_coordinates := PackedVector2Array()
	var metric_coordinates := PackedVector2Array()
	vertices.resize(sections * columns)
	normals.resize(sections * columns)
	texture_coordinates.resize(sections * columns)
	metric_coordinates.resize(sections * columns)
	var along := float(chunk.start_distance_m)
	var previous_centre := Vector3.INF
	var half_width := float(direction.half_width_m)
	for section in range(sections):
		for column in range(columns):
			var vertex_index := section * columns + column
			var source_index := vertex_index * stride
			vertices[vertex_index] = Vector3(values[source_index], values[source_index + 1], values[source_index + 2])
			normals[vertex_index] = Vector3(values[source_index + 3], values[source_index + 4], values[source_index + 5])
			texture_coordinates[vertex_index] = Vector2(float(column) / (columns - 1), float(section) / (sections - 1))
		var center_index := section * columns + columns / 2
		if previous_centre != Vector3.INF:
			along += Vector2(vertices[center_index].x - previous_centre.x, vertices[center_index].z - previous_centre.z).length()
		previous_centre = vertices[center_index]
		for column in range(columns):
			metric_coordinates[section * columns + column] = Vector2(lerpf(-half_width, half_width, float(column) / (columns - 1)), along)
		if centerline.is_empty() or centerline[-1].distance_to(vertices[center_index]) > 0.01:
			centerline.append(vertices[center_index])
	var indices := PackedInt32Array()
	indices.resize((sections - 1) * (columns - 1) * 6)
	var cursor := 0
	for section in range(sections - 1):
		for column in range(columns - 1):
			var corner := section * columns + column
			for index in [corner, corner + columns + 1, corner + 1, corner, corner + columns, corner + columns + 1]:
				indices[cursor] = index
				cursor += 1
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = texture_coordinates
	arrays[Mesh.ARRAY_TEX_UV2] = metric_coordinates
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	add_verge_surface(mesh, vertices, sections, columns, first_chunk, last_chunk)
	var material := StandardMaterial3D.new()
	material.roughness = 0.92
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	material.texture_repeat = false
	var image := Image.new()
	var error := image.load_png_from_buffer(FileAccess.get_file_as_bytes(str(chunk.texture_file)))
	assert(error == OK and not image.is_empty(), "Detailed-road texture could not be loaded")
	image.generate_mipmaps()
	material.albedo_texture = ImageTexture.create_from_image(image)
	photo_materials.append(material)
	mesh.surface_set_material(0, material if photo_mode else stylised_material)
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	add_child(instance)
	mesh_instances.append(instance)
	var body := StaticBody3D.new()
	body.collision_layer = 2
	body.collision_mask = 0
	body.name = "RoadCollision"
	var collision := CollisionShape3D.new()
	collision.shape = mesh.create_trimesh_shape()
	body.add_child(collision)
	add_child(body)
	collision_bodies.append(body)
	chunk_count += 1
	vertex_count += vertices.size() if chunk_count == 1 else vertices.size() - columns
	triangle_count += (sections - 1) * (columns - 1) * 2

func set_photo_mode(enabled: bool) -> void:
	photo_mode = enabled
	for index in range(mesh_instances.size()):
		mesh_instances[index].mesh.surface_set_material(0, photo_materials[index] if photo_mode else stylised_material)

func verge_point(edge: Vector3, outward: Vector3, max_width := VERGE_MAX_WIDTH) -> Vector3:
	var width := max_width
	var distance := minf(VERGE_MIN_WIDTH, max_width)
	while distance < max_width + 0.01:
		if absf(edge.y - terrain.height_at(edge + outward * distance)) <= distance * VERGE_TARGET_GRADE:
			width = distance
			break
		distance += 0.5
	var outer := edge + outward * width
	outer.y = terrain.height_at(outer) - VERGE_BURY
	return outer

func add_strip(surface: SurfaceTool, inner: PackedVector3Array, outer: PackedVector3Array) -> void:
	for index in range(inner.size() - 1):
		for triangle in [[inner[index], outer[index], inner[index + 1]], [inner[index + 1], outer[index], outer[index + 1]]]:
			# Godot treats clockwise-from-above as the front face; collision ignores back faces.
			if (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]).y > 0:
				triangle.reverse()
			for vertex in triangle:
				surface.add_vertex(vertex)

func add_verge_surface(mesh: ArrayMesh, vertices: PackedVector3Array, sections: int, columns: int, first_chunk: bool, last_chunk: bool) -> void:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	var curvature := PackedVector3Array()
	for section in range(sections):
		var before := vertices[maxi(section - 8, 0) * columns + columns / 2]
		var here := vertices[section * columns + columns / 2]
		var after := vertices[mini(section + 8, sections - 1) * columns + columns / 2]
		var incoming := Vector3(here.x - before.x, 0, here.z - before.z)
		var outgoing := Vector3(after.x - here.x, 0, after.z - here.z)
		var length := (incoming.length() + outgoing.length()) * 0.5
		curvature.append((outgoing.normalized() - incoming.normalized()) / length if length > 0.01 and incoming.length() > 0.01 and outgoing.length() > 0.01 else Vector3.ZERO)
	for edge_column in [0, columns - 1]:
		var neighbour_column: int = 1 if edge_column == 0 else columns - 2
		var inner := PackedVector3Array()
		var outer := PackedVector3Array()
		for section in range(sections):
			var edge := vertices[section * columns + edge_column]
			var outward := edge - vertices[section * columns + neighbour_column]
			outward.y = 0
			outward = outward.normalized()
			var max_width := VERGE_MAX_WIDTH
			# On the inside of a bend, stop short of the centre of curvature so strips cannot fold.
			if outward.dot(curvature[section]) > 0.0001:
				var centre := vertices[section * columns + columns / 2]
				var radius := 1.0 / curvature[section].length()
				max_width = clampf(0.6 * (radius - Vector2(edge.x - centre.x, edge.z - centre.z).length()), 1.0, VERGE_MAX_WIDTH)
			inner.append(edge)
			outer.append(verge_point(edge, outward, max_width))
		add_strip(surface, inner, outer)
	for end in ([0] if first_chunk else []) + ([sections - 1] if last_chunk else []):
		var neighbour: int = 1 if end == 0 else sections - 2
		var forward := vertices[end * columns + columns / 2] - vertices[neighbour * columns + columns / 2]
		forward.y = 0
		var inner := PackedVector3Array()
		var outer := PackedVector3Array()
		for column in range(columns):
			inner.append(vertices[end * columns + column])
			outer.append(verge_point(vertices[end * columns + column], forward.normalized()))
		add_strip(surface, inner, outer)
	surface.generate_normals()
	surface.commit(mesh)
	mesh.surface_set_material(mesh.get_surface_count() - 1, verge_material)