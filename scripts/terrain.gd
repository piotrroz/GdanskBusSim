extends Node3D

var metadata: Dictionary
var heights := PackedFloat32Array()
var columns := 0
var rows := 0
var spacing := 5.0
var grid_start := Vector2.ZERO
var orthophoto_metadata: Dictionary
var orthophoto_texture: Texture2D
var ground_material: StandardMaterial3D
var stylised_material: ShaderMaterial
var photo_mode := false
var chunks: Array[MeshInstance3D] = []

const STYLISED_SHADER := """
shader_type spatial;
uniform sampler2D landcover : source_color, filter_linear_mipmap, repeat_disable;
uniform bool has_landcover = false;
uniform vec3 grass_light : source_color = vec3(0.56, 0.64, 0.40);
uniform vec3 grass_dark : source_color = vec3(0.40, 0.51, 0.31);
uniform vec3 paved : source_color = vec3(0.67, 0.66, 0.63);
uniform vec3 paved_dark : source_color = vec3(0.52, 0.53, 0.54);
varying vec3 world_position;
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float value_noise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x), mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
}
void vertex() { world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }
void fragment() {
	vec2 p = world_position.xz;
	vec3 grass = mix(grass_dark, grass_light, value_noise(p * 0.07) * 0.65 + value_noise(p * 0.5) * 0.35);
	vec3 pavement = paved * (0.94 + 0.06 * value_noise(p * 0.3));
	float vegetated = 1.0;
	if (has_landcover) {
		// Heavily blurred imagery only decides paved vs green; photo detail never reaches the screen.
		vec3 cover = textureLod(landcover, UV, 3.5).rgb;
		// Excess green, calibrated on the bundled 2021 imagery (median 0.014, 75th percentile 0.041).
		vegetated = smoothstep(0.012, 0.045, 2.0 * cover.g - cover.r - cover.b);
		float brightness = (cover.r + cover.g + cover.b) / 3.0;
		pavement = mix(paved_dark, paved, smoothstep(0.30, 0.46, brightness)) * (0.95 + 0.05 * value_noise(p * 0.3));
	}
	ALBEDO = mix(pavement, grass, vegetated);
	ROUGHNESS = 0.96;
}
"""

func orthophoto_fits(description: Dictionary) -> bool:
	var aligned := absf(float(description.origin_lon) - float(metadata.origin_lon)) < 0.000000001 and absf(float(description.origin_lat) - float(metadata.origin_lat)) < 0.000000001
	aligned = aligned and absf(float(description.grid_x) - grid_start.x) < 0.001 and absf(float(description.grid_z) - grid_start.y) < 0.001
	aligned = aligned and absf(float(description.width_m) - (columns - 1) * spacing) < 0.001 and absf(float(description.height_m) - (rows - 1) * spacing) < 0.001
	return aligned

func load_orthophoto() -> void:
	if not FileAccess.file_exists("res://data/orthophoto_108.json"):
		return
	var description: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/orthophoto_108.json"))
	if not orthophoto_fits(description):
		push_warning("Orthophoto crop does not match terrain. Re-run tools/import_orthophotos.py.")
		return
	var image := Image.load_from_file(str(description.texture_file))
	if image == null or image.is_empty():
		push_warning("Orthophoto image could not be loaded; using plain terrain.")
		return
	if image.get_width() != int(description.width) or image.get_height() != int(description.height):
		push_warning("Orthophoto image dimensions do not match metadata; using plain terrain.")
		return
	image.generate_mipmaps()
	orthophoto_metadata = description
	orthophoto_texture = ImageTexture.create_from_image(image)

func set_photo_mode(enabled: bool) -> void:
	photo_mode = enabled and is_instance_valid(orthophoto_texture)
	for chunk in chunks:
		chunk.material_override = ground_material if photo_mode else stylised_material

func load_data() -> void:
	metadata = JSON.parse_string(FileAccess.get_file_as_string("res://data/terrain_108.json"))
	columns = int(metadata.columns)
	rows = int(metadata.rows)
	spacing = float(metadata.spacing_m)
	grid_start = Vector2(float(metadata.grid_x), float(metadata.grid_z))
	heights = FileAccess.get_file_as_bytes(str(metadata.height_file)).to_float32_array()
	assert(heights.size() == columns * rows, "Terrain dimensions do not match height data")

func contains(location: Vector3) -> bool:
	var grid := (Vector2(location.x, location.z) - grid_start) / spacing
	return grid.x >= 0 and grid.y >= 0 and grid.x < columns - 1 and grid.y < rows - 1

# The 5 m grid smooths embankments across narrow carriageways; keep it below the road.
func carve_below(surface: PackedVector3Array, clearance: float) -> int:
	var lowered := {}
	for point in surface:
		var column := clampi(floori((point.x - grid_start.x) / spacing), 0, columns - 2)
		var row := clampi(floori((point.z - grid_start.y) / spacing), 0, rows - 2)
		var limit := point.y - clearance
		for index in [row * columns + column, row * columns + column + 1, (row + 1) * columns + column, (row + 1) * columns + column + 1]:
			if heights[index] > limit:
				heights[index] = limit
				lowered[index] = true
	return lowered.size()

func height_at(location: Vector3) -> float:
	var grid := (Vector2(location.x, location.z) - grid_start) / spacing
	var column := clampi(floori(grid.x), 0, columns - 2)
	var row := clampi(floori(grid.y), 0, rows - 2)
	var horizontal := clampf(grid.x - column, 0, 1)
	var vertical := clampf(grid.y - row, 0, 1)
	var northwest := heights[row * columns + column]
	var northeast := heights[row * columns + column + 1]
	var southwest := heights[(row + 1) * columns + column]
	var southeast := heights[(row + 1) * columns + column + 1]
	if horizontal + vertical <= 1.0:
		return northwest + (northeast - northwest) * horizontal + (southwest - northwest) * vertical
	return southeast + (southwest - southeast) * (1.0 - horizontal) + (northeast - southeast) * (1.0 - vertical)

func build() -> void:
	var material := StandardMaterial3D.new()
	material.albedo_color = Color("829a70")
	material.roughness = 1.0
	material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	material.texture_repeat = false
	ground_material = material
	load_orthophoto()
	material.albedo_texture = orthophoto_texture
	material.albedo_color = Color.WHITE if is_instance_valid(orthophoto_texture) else Color("829a70")
	var shader := Shader.new()
	shader.code = STYLISED_SHADER
	stylised_material = ShaderMaterial.new()
	stylised_material.shader = shader
	stylised_material.set_shader_parameter("has_landcover", is_instance_valid(orthophoto_texture))
	stylised_material.set_shader_parameter("landcover", orthophoto_texture)
	for start_row in range(0, rows - 1, 64):
		for start_column in range(0, columns - 1, 64):
			var width := mini(65, columns - start_column)
			var depth := mini(65, rows - start_row)
			var vertices := PackedVector3Array()
			var normals := PackedVector3Array()
			var texture_coordinates := PackedVector2Array()
			var indices := PackedInt32Array()
			for row in range(depth):
				for column in range(width):
					var grid_column := start_column + column
					var grid_row := start_row + row
					var elevation := heights[grid_row * columns + grid_column]
					vertices.append(Vector3(grid_start.x + grid_column * spacing, elevation, grid_start.y + grid_row * spacing))
					texture_coordinates.append(Vector2(float(grid_column) / (columns - 1), float(grid_row) / (rows - 1)))
					var left := heights[grid_row * columns + maxi(0, grid_column - 1)]
					var right := heights[grid_row * columns + mini(columns - 1, grid_column + 1)]
					var north := heights[maxi(0, grid_row - 1) * columns + grid_column]
					var south := heights[mini(rows - 1, grid_row + 1) * columns + grid_column]
					normals.append(Vector3(left - right, spacing * 2, north - south).normalized())
			for row in range(depth - 1):
				for column in range(width - 1):
					var corner := row * width + column
					indices.append_array(PackedInt32Array([corner, corner + 1, corner + width, corner + 1, corner + width + 1, corner + width]))
			var arrays := []
			arrays.resize(Mesh.ARRAY_MAX)
			arrays[Mesh.ARRAY_VERTEX] = vertices
			arrays[Mesh.ARRAY_NORMAL] = normals
			arrays[Mesh.ARRAY_TEX_UV] = texture_coordinates
			arrays[Mesh.ARRAY_INDEX] = indices
			var mesh := ArrayMesh.new()
			mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
			var instance := MeshInstance3D.new()
			instance.mesh = mesh
			instance.material_override = ground_material if photo_mode else stylised_material
			add_child(instance)
			chunks.append(instance)
			instance.create_trimesh_collision()