extends Node3D

# Runtime for data/roads_108 schema 2: OSM carriageway network tiles, bridge/tunnel floors, verges and tunnel covers.
const SCHEMA := 2
const VERGE_GRADE := 0.25
const VERGE_BURY := 0.06
const PARAPET_HEIGHT := 1.0
const ROUTE_CELL := 8.0
const SHADER := """
shader_type spatial;
render_mode cull_back;
uniform bool photo = false;
uniform sampler2D ortho : source_color, filter_linear_mipmap_anisotropic;
uniform vec2 ortho_origin;
uniform vec2 ortho_size = vec2(1.0);
uniform vec3 asphalt : source_color = vec3(0.30, 0.31, 0.32);
uniform vec3 marking : source_color = vec3(0.93, 0.93, 0.89);
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
	vec2 p = world_position.xz;
	if (photo) {
		ALBEDO = texture(ortho, (p - ortho_origin) / ortho_size).rgb;
		ROUGHNESS = 0.92;
	} else {
		// UV2: lateral offset and distance along the owning carriageway; UV: half width, lanes + 10 * flags; COLOR.r: junction fade.
		float x = UV2.x;
		float along = UV2.y;
		float hw = UV.x;
		float lanes = max(mod(UV.y + 0.5, 10.0) - 0.5, 1.0);
		int flags = int(floor((UV.y + 0.5) / 10.0));
		float blur = fwidth(x) + 0.01;
		vec3 colour = asphalt * (0.9 + 0.12 * value_noise(p * 0.35) + 0.08 * value_noise(p * 3.1));
		float dash = step(fract(along / 12.0), 0.333);
		float paint = 0.0;
		if ((flags & 2) != 0) {
			paint = max(paint, band(abs(x), hw - 0.35, 0.06, blur));
		}
		if ((flags & 1) != 0) {
			for (int k = 1; k < 8; k++) {
				if (float(k) >= lanes) { break; }
				paint = max(paint, band(x, -hw + 2.0 * hw * float(k) / lanes, 0.06, blur) * dash);
			}
		} else {
			float per_side = floor(lanes / 2.0);
			if ((flags & 4) != 0) {
				float centre = band(x, 0.0, 0.06, blur) * dash;
				if ((flags & 8) != 0) {
					centre = max(band(x, -0.15, 0.06, blur), band(x, 0.15, 0.06, blur));
				}
				paint = max(paint, centre);
			}
			for (int k = 1; k < 4; k++) {
				if (float(k) >= per_side) { break; }
				float offset = hw * float(k) / per_side;
				paint = max(paint, max(band(x, offset, 0.06, blur), band(x, -offset, 0.06, blur)) * dash);
			}
		}
		paint *= clamp(COLOR.r, 0.0, 1.0) * (0.8 + 0.2 * value_noise(p * 6.0));
		ALBEDO = mix(colour, marking, paint);
		ROUGHNESS = mix(0.92, 0.7, paint);
	}
}
"""

var metadata: Dictionary
var available := false
var terrain: Node3D
var centerline := PackedVector3Array()
var route_edges := PackedVector2Array()
var building_clips: Dictionary = {}
var included_way_ids: Dictionary = {}
var mesh_instances: Array[MeshInstance3D] = []
var collision_bodies: Array[StaticBody3D] = []
var cover_instances: Array[MeshInstance3D] = []
var road_material: ShaderMaterial
var verge_material: StandardMaterial3D
var photo_mode := false
var carved_vertices := 0
var blob := PackedByteArray()
var route_grid: Dictionary = {}

func load_data(shape_id: String, terrain_metadata: Dictionary) -> void:
	var path := "res://data/roads_108/roads_108.json"
	if not FileAccess.file_exists(path):
		return
	var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(path))
	if int(data.get("schema_version", 0)) != SCHEMA:
		push_warning("Road network schema mismatch; rerun tools/import_road_network.py.")
		return
	var aligned := absf(float(data.origin_lon) - float(terrain_metadata.origin_lon)) < 0.000000001 and absf(float(data.origin_lat) - float(terrain_metadata.origin_lat)) < 0.000000001
	for key in data.terrain_grid:
		aligned = aligned and data.terrain_grid[key] == terrain_metadata[key]
	if not aligned or not data.route_samples.has(shape_id):
		push_warning("Road network does not match terrain or route; using legacy roads.")
		return
	metadata = data
	blob = FileAccess.get_file_as_bytes(str(data.binary))
	for sample in data.route_samples[shape_id]:
		centerline.append(Vector3(float(sample[0]), float(sample[1]), float(sample[2])))
		route_edges.append(Vector2(float(sample[3]), float(sample[4])))
		var key := Vector2i(floori(float(sample[0]) / ROUTE_CELL), floori(float(sample[2]) / ROUTE_CELL))
		if not route_grid.has(key):
			route_grid[key] = PackedInt32Array()
		route_grid[key].append(centerline.size() - 1)
	for identifier in data.building_clips:
		building_clips[str(identifier)] = data.building_clips[identifier]
	for identifier in data.included_way_ids:
		included_way_ids[str(int(identifier))] = true
	available = true

func floats(info: Dictionary, stride: int) -> PackedFloat32Array:
	var start := int(info.vertex_offset)
	return blob.slice(start, start + int(info.vertex_count) * stride * 4).to_float32_array()

func ints(info: Dictionary) -> PackedInt32Array:
	var start := int(info.index_offset)
	return blob.slice(start, start + int(info.index_count) * 4).to_int32_array()

# Road and structure-floor vertices, used to keep the coarse terrain below the carriageway.
func surface_points() -> PackedVector3Array:
	var points := PackedVector3Array()
	for tile in metadata.tiles:
		for name in ["road", "layered"]:
			if tile.surfaces.has(name):
				var values := floats(tile.surfaces[name], 12)
				for offset in range(0, values.size(), 12):
					points.append(Vector3(values[offset], values[offset + 1], values[offset + 2]))
	return points

# Height of the bus route surface near a location; NAN when the location is away from the route.
func route_height(location: Vector3, max_distance := 16.0) -> float:
	var key := Vector2i(floori(location.x / ROUTE_CELL), floori(location.z / ROUTE_CELL))
	var best := max_distance * max_distance
	var height := NAN
	var reach := ceili(max_distance / ROUTE_CELL)
	for dx in range(-reach, reach + 1):
		for dz in range(-reach, reach + 1):
			for index in route_grid.get(key + Vector2i(dx, dz), PackedInt32Array()):
				var sample := centerline[index]
				var distance := Vector2(sample.x - location.x, sample.z - location.z).length_squared()
				if distance < best:
					best = distance
					height = sample.y
	return height

func build(ground: Node3D) -> void:
	if not available:
		return
	terrain = ground
	var shader := Shader.new()
	shader.code = SHADER
	road_material = ShaderMaterial.new()
	road_material.shader = shader
	if is_instance_valid(terrain.orthophoto_texture):
		road_material.set_shader_parameter("ortho", terrain.orthophoto_texture)
		road_material.set_shader_parameter("ortho_origin", terrain.grid_start)
		road_material.set_shader_parameter("ortho_size", Vector2((terrain.columns - 1) * terrain.spacing, (terrain.rows - 1) * terrain.spacing))
	verge_material = StandardMaterial3D.new()
	verge_material.albedo_color = Color("a9a99c")
	verge_material.roughness = 1.0
	verge_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	for tile in metadata.tiles:
		build_tile(tile.surfaces)
	build_parapets()
	set_photo_mode(photo_mode)

func road_arrays(info: Dictionary) -> Array:
	var values := floats(info, 12)
	var count := int(info.vertex_count)
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var attributes := PackedVector2Array()
	var metric := PackedVector2Array()
	var colours := PackedColorArray()
	vertices.resize(count)
	normals.resize(count)
	attributes.resize(count)
	metric.resize(count)
	colours.resize(count)
	for index in range(count):
		var offset := index * 12
		vertices[index] = Vector3(values[offset], values[offset + 1], values[offset + 2])
		normals[index] = Vector3(values[offset + 3], values[offset + 4], values[offset + 5])
		metric[index] = Vector2(values[offset + 6], values[offset + 7])
		attributes[index] = Vector2(values[offset + 8], values[offset + 9] + 10.0 * values[offset + 10])
		colours[index] = Color(values[offset + 11], 0, 0)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = attributes
	arrays[Mesh.ARRAY_TEX_UV2] = metric
	arrays[Mesh.ARRAY_COLOR] = colours
	arrays[Mesh.ARRAY_INDEX] = ints(info)
	return arrays

# Verge heights blend from the road edge to the (carved) terrain, widening on embankments to keep a drivable grade.
func verge_arrays(info: Dictionary) -> Array:
	var values := floats(info, 4)
	var vertices := PackedVector3Array()
	vertices.resize(int(info.vertex_count))
	var limit := float(metadata.verge_max_m)
	for index in range(vertices.size()):
		var offset := index * 4
		var location := Vector3(values[offset], 0, values[offset + 1])
		var edge := values[offset + 2]
		var ground: float = terrain.height_at(location) - VERGE_BURY
		var width := clampf(absf(edge - ground) / VERGE_GRADE, 1.5, limit)
		var blend := smoothstep(0.0, 1.0, clampf(values[offset + 3] / width, 0.0, 1.0))
		location.y = lerpf(edge, ground, blend)
		vertices[index] = location
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_INDEX] = ints(info)
	return arrays

func cover_arrays(info: Dictionary) -> Array:
	var values := floats(info, 3)
	var vertices := PackedVector3Array()
	var coordinates := PackedVector2Array()
	var extent := Vector2((terrain.columns - 1) * terrain.spacing, (terrain.rows - 1) * terrain.spacing)
	for offset in range(0, values.size(), 3):
		vertices.append(Vector3(values[offset], values[offset + 1], values[offset + 2]))
		coordinates.append((Vector2(values[offset], values[offset + 2]) - terrain.grid_start) / extent)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_TEX_UV] = coordinates
	arrays[Mesh.ARRAY_INDEX] = ints(info)
	return arrays

func with_normals(arrays: Array) -> Array:
	var tool := SurfaceTool.new()
	tool.create_from_arrays(arrays)
	tool.generate_normals()
	return tool.commit_to_arrays()

func build_tile(surfaces: Dictionary) -> void:
	var mesh := ArrayMesh.new()
	for name in ["road", "layered"]:
		if surfaces.has(name):
			mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, road_arrays(surfaces[name]))
			mesh.surface_set_material(mesh.get_surface_count() - 1, road_material)
	if surfaces.has("verge"):
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, with_normals(verge_arrays(surfaces.verge)))
		mesh.surface_set_material(mesh.get_surface_count() - 1, verge_material)
	if mesh.get_surface_count() > 0:
		var instance := MeshInstance3D.new()
		instance.mesh = mesh
		add_child(instance)
		mesh_instances.append(instance)
		add_collision(mesh, 2)
	if surfaces.has("cover"):
		var cover := ArrayMesh.new()
		cover.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, with_normals(cover_arrays(surfaces.cover)))
		var instance := MeshInstance3D.new()
		instance.mesh = cover
		add_child(instance)
		cover_instances.append(instance)
		add_collision(cover, 1)

func add_collision(mesh: Mesh, layer: int) -> void:
	var body := StaticBody3D.new()
	body.collision_layer = layer
	body.collision_mask = 0
	var shape := CollisionShape3D.new()
	shape.shape = mesh.create_trimesh_shape()
	body.add_child(shape)
	add_child(body)
	collision_bodies.append(body)

# Bridge railings along both deck edges; walls face outward on both sides so collision works either way.
func build_parapets() -> void:
	var tool := SurfaceTool.new()
	tool.begin(Mesh.PRIMITIVE_TRIANGLES)
	var count := 0
	for structure in metadata.structures:
		if str(structure.kind) != "bridge" or structure.centre.size() < 2:
			continue
		var centre := PackedVector3Array()
		for point in structure.centre:
			centre.append(Vector3(float(point[0]), float(point[1]), float(point[2])))
		for side in [-1.0, 1.0]:
			for index in range(centre.size() - 1):
				var tangent := Vector3(centre[index + 1].x - centre[index].x, 0, centre[index + 1].z - centre[index].z).normalized()
				var outward: Vector3 = tangent.cross(Vector3.UP) * side
				var offset: Vector3 = outward * float(structure.half_width)
				var start := centre[index] + offset
				var finish := centre[index + 1] + offset
				for wall in [[start, finish], [finish + outward * 0.25, start + outward * 0.25]]:
					var a: Vector3 = wall[0]
					var b: Vector3 = wall[1]
					for vertex in [a, b + Vector3.UP * PARAPET_HEIGHT, b, a, a + Vector3.UP * PARAPET_HEIGHT, b + Vector3.UP * PARAPET_HEIGHT]:
						tool.add_vertex(vertex)
				count += 1
	if count == 0:
		return
	tool.generate_normals()
	var mesh := tool.commit()
	var paint := StandardMaterial3D.new()
	paint.albedo_color = Color("b9bab3")
	paint.cull_mode = BaseMaterial3D.CULL_DISABLED
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.material_override = paint
	# Railings are obstacles like buildings, not part of the driving surface.
	var railings := Node3D.new()
	railings.name = "BridgeRailings"
	get_parent().add_child(railings)
	railings.add_child(instance)
	var body := StaticBody3D.new()
	body.collision_mask = 0
	var shape := CollisionShape3D.new()
	shape.shape = mesh.create_trimesh_shape()
	body.add_child(shape)
	railings.add_child(body)

func set_photo_mode(enabled: bool) -> void:
	photo_mode = enabled
	if road_material:
		road_material.set_shader_parameter("photo", enabled and is_instance_valid(terrain.orthophoto_texture))
	for instance in cover_instances:
		instance.material_override = terrain.ground_material if enabled else terrain.stylised_material
