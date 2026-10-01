extends RefCounted

# Body shell for OMSI 2 "Urbino IV" repaints. Atlas rectangles were measured on the template
# (side bands are 343 px/m with the 5.9 m wheelbase arches), so any repaint of that template fits.
const PX_PER_M := 343.0
const SIDE_END := 5.86
const HALF_WIDTH := 1.275
const NOSE_HALF_WIDTH := 1.135
const NOSE := 6.0
const SKIRT := 0.33
const BELT := 1.28
const SILL := 1.86
const CANT := 2.72
const ROOF := 3.30
const FRONT_MASK_TOP := 1.115
const REAR_PANEL_TOP := 1.61
const REAR_CAP_BOTTOM := 2.79
# Atlas rows for each side band, plus the pixel column where the band starts (rear on the right, front on the left).
const RIGHT := {"lower": Vector2(696, 1016), "waist": Vector2(344, 540), "top": Vector2(92, 288), "start": 51.0, "sign": -1.0}
const LEFT := {"lower": Vector2(1716, 2044), "waist": Vector2(1350, 1546), "top": Vector2(1122, 1332), "start": 11.0, "sign": 1.0}
const FRONT_MASK := Rect2(64, 2658, 824, 259)
const REAR_PANEL := Rect2(927, 2584, 857, 434)
const REAR_CAP := Rect2(918, 2125, 883, 175)
const ROOF_TOP := Rect2(13, 3380, 2546, 683)
const WHEEL_COVER_CENTRE := Vector2(2925, 3920)
const WHEEL_COVER_RADIUS := 140.0
# Door openings in the right-hand lower panel, converted from the template's black gaps.
const DOORS := [Vector2(-5.55, -4.15), Vector2(-1.28, 0.11), Vector2(3.53, 4.90)]
const SPEED_LIMITS := [70.0, 60.0, 65.0, 80.0]
const RANGES_KM := [75.0, 100.0, 150.0, 200.0, 250.0]
const CUTOUT_LIMIT := 5.45
const CABIN_FRONT := 4.05
const CUTOUT_SHADER := preload("res://shaders/livery_cutout.gdshader")

var name := ""
var texture_paths := {}
var textures := {}
var settings := {}
var atlas_size := Vector2(4096, 4096)

static func find_default() -> String:
	var folder := "res://Vehicles/Urbino IV"
	var directory := DirAccess.open(folder)
	if directory == null:
		return ""
	for file in directory.get_files():
		if file.get_extension().to_lower() == "cti":
			return folder.path_join(file)
	return ""

static func parse_cti(text: String) -> Dictionary:
	var items := {}
	var values := {}
	var lines := text.replace("\r", "").split("\n")
	var title := ""
	for index in range(lines.size()):
		var line := lines[index].strip_edges()
		if line == "[item]" and index + 3 < lines.size():
			title = lines[index + 1].strip_edges()
			items[lines[index + 2].strip_edges()] = lines[index + 3].strip_edges().replace("\\", "/")
		elif line == "[setvar]" and index + 2 < lines.size() and lines[index + 2].strip_edges().is_valid_int():
			values[lines[index + 1].strip_edges()] = lines[index + 2].strip_edges().to_int()
	return {"name": title, "items": items, "settings": values}

static func load_texture(path: String) -> Texture2D:
	if ResourceLoader.exists(path):
		return load(path)
	var image := Image.load_from_file(path)
	if image == null or image.is_empty():
		return null
	image.generate_mipmaps()
	return ImageTexture.create_from_image(image)

func load_repaint(cti_path: String) -> bool:
	if cti_path.is_empty() or not FileAccess.file_exists(cti_path):
		return false
	var parsed := parse_cti(FileAccess.get_file_as_string(cti_path))
	name = parsed.name
	settings = parsed.settings
	for key in parsed.items:
		texture_paths[key] = cti_path.get_base_dir().path_join(parsed.items[key])
	if texture("repaint_body") == null:
		return false
	atlas_size = Vector2(textures.repaint_body.get_size())
	return true

# Interior textures in the repaint are large and unused by this shell, so load only on demand.
func texture(key: String) -> Texture2D:
	if not textures.has(key) and texture_paths.has(key):
		var loaded := load_texture(texture_paths[key])
		if loaded:
			textures[key] = loaded
	return textures.get(key)

func setting(key: String, fallback: int) -> int:
	return int(settings.get(key, fallback))

func top_speed_kmh() -> float:
	return SPEED_LIMITS[clampi(setting("EV_kaganiec", 0), 0, SPEED_LIMITS.size() - 1)]

func range_km() -> float:
	return RANGES_KM[clampi(setting("EV_zasieg", 4), 0, RANGES_KM.size() - 1)]

func uv(pixel: Vector2) -> Vector2:
	return pixel / atlas_size

# Corners in outside view order: bottom-left, bottom-right, top-right, top-left (clockwise front faces).
func quad(tool: SurfaceTool, corners: Array, pixels: Array) -> void:
	for index in [0, 3, 2, 0, 2, 1]:
		tool.set_uv(uv(pixels[index]))
		tool.add_vertex(corners[index])

func band_column(side: Dictionary, z: float) -> float:
	return side.start + (SIDE_END + side.sign * z) * PX_PER_M

func side_strip(tool: SurfaceTool, side: Dictionary, rows: Vector2, bottom: float, top: float, z_from: float, z_to: float) -> void:
	var x: float = -side.sign * HALF_WIDTH
	# Outside view: on the right (+x) the front is on the viewer's right, on the left side it is on the left.
	var near_z := maxf(z_from, z_to) if side.sign < 0 else minf(z_from, z_to)
	var far_z := minf(z_from, z_to) if side.sign < 0 else maxf(z_from, z_to)
	quad(tool, [Vector3(x, bottom, near_z), Vector3(x, bottom, far_z), Vector3(x, top, far_z), Vector3(x, top, near_z)],
		[Vector2(band_column(side, near_z), rows.y), Vector2(band_column(side, far_z), rows.y),
		Vector2(band_column(side, far_z), rows.x), Vector2(band_column(side, near_z), rows.x)])
	for end in [-1.0, 1.0]:
		var edge: float = end * SIDE_END
		if absf(z_from - edge) > 0.01 and absf(z_to - edge) > 0.01:
			continue
		# Chamfered corner reuses the last few texels of the band.
		var column: float = band_column(side, edge) - side.sign * end * 12.0
		var inner := Vector3(x * NOSE_HALF_WIDTH / HALF_WIDTH, 0, end * NOSE)
		var outer := Vector3(x, 0, edge)
		var left_point := outer if (end < 0) == (side.sign < 0) else inner
		var right_point := inner if left_point == outer else outer
		quad(tool, [left_point + Vector3.UP * bottom, right_point + Vector3.UP * bottom, right_point + Vector3.UP * top, left_point + Vector3.UP * top],
			[Vector2(column, rows.y), Vector2(column, rows.y), Vector2(column, rows.x), Vector2(column, rows.x)])

func panel(tool: SurfaceTool, region: Rect2, z: float, bottom: float, top: float, facing_front: bool) -> void:
	var w := NOSE_HALF_WIDTH
	var a := Vector3(w if facing_front else -w, bottom, z)
	var b := Vector3(-a.x, bottom, z)
	quad(tool, [a, b, Vector3(b.x, top, z), Vector3(a.x, top, z)],
		[Vector2(region.position.x, region.end.y), region.end, Vector2(region.end.x, region.position.y), region.position])

func build_shell(parent: Node3D, glass: Material, interior: Material) -> void:
	var paint := StandardMaterial3D.new()
	paint.albedo_texture = textures.repaint_body
	paint.roughness = 0.38
	paint.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	# The template marks wheel arches with pure black, which window paint also uses, so cut out only lower panels.
	var cutout := ShaderMaterial.new()
	cutout.shader = CUTOUT_SHADER
	cutout.set_shader_parameter("albedo_texture", textures.repaint_body)
	var tool := SurfaceTool.new()
	tool.begin(Mesh.PRIMITIVE_TRIANGLES)
	var cut_tool := SurfaceTool.new()
	cut_tool.begin(Mesh.PRIMITIVE_TRIANGLES)
	var door_side := [Vector2(-SIDE_END, DOORS[0].x), Vector2(DOORS[0].y, DOORS[1].x), Vector2(DOORS[1].y, DOORS[2].x), Vector2(DOORS[2].y, SIDE_END)]
	for side in [RIGHT, LEFT]:
		var spans: Array = door_side if side.sign < 0 else [Vector2(-SIDE_END, SIDE_END)]
		for span in spans:
			# Corner pieces stay opaque: the mask wraps there and the template leaves them black.
			var cut_from := maxf(span.x, -CUTOUT_LIMIT)
			var cut_to := minf(span.y, CUTOUT_LIMIT)
			if cut_to > cut_from:
				side_strip(cut_tool, side, side.lower, SKIRT, BELT, cut_from, cut_to)
			if span.x < -CUTOUT_LIMIT:
				side_strip(tool, side, side.lower, SKIRT, BELT, span.x, minf(span.y, -CUTOUT_LIMIT))
			if span.y > CUTOUT_LIMIT:
				side_strip(tool, side, side.lower, SKIRT, BELT, maxf(span.x, CUTOUT_LIMIT), span.y)
			side_strip(tool, side, side.waist, BELT, SILL, span.x, span.y)
		side_strip(tool, side, side.top, CANT, ROOF, -SIDE_END, SIDE_END)
	panel(tool, FRONT_MASK, -NOSE, SKIRT, FRONT_MASK_TOP, true)
	panel(tool, REAR_PANEL, NOSE, SKIRT, REAR_PANEL_TOP, false)
	panel(tool, REAR_CAP, NOSE, REAR_CAP_BOTTOM, ROOF, false)
	# Roof: front of the bus at the left of the atlas region.
	var roof_corners := [Vector3(-HALF_WIDTH, ROOF, -SIDE_END), Vector3(HALF_WIDTH, ROOF, -SIDE_END), Vector3(HALF_WIDTH, ROOF, SIDE_END), Vector3(-HALF_WIDTH, ROOF, SIDE_END)]
	quad(tool, [roof_corners[3], roof_corners[2], roof_corners[1], roof_corners[0]],
		[ROOF_TOP.end, Vector2(ROOF_TOP.end.x, ROOF_TOP.position.y), ROOF_TOP.position, Vector2(ROOF_TOP.position.x, ROOF_TOP.end.y)])
	for end in [-1.0, 1.0]:
		var edge := Vector2(ROOF_TOP.position.x if end < 0 else ROOF_TOP.end.x, ROOF_TOP.get_center().y)
		var nose := [Vector3(-NOSE_HALF_WIDTH, ROOF, end * NOSE), Vector3(NOSE_HALF_WIDTH, ROOF, end * NOSE), Vector3(HALF_WIDTH, ROOF, end * SIDE_END), Vector3(-HALF_WIDTH, ROOF, end * SIDE_END)]
		if end < 0:
			nose = [nose[3], nose[2], nose[1], nose[0]]
		quad(tool, nose, [edge, edge, edge, edge])
	tool.generate_normals()
	add_mesh(parent, tool.commit(), paint)
	cut_tool.generate_normals()
	add_mesh(parent, cut_tool.commit(), cutout)

	var glazing := SurfaceTool.new()
	glazing.begin(Mesh.PRIMITIVE_TRIANGLES)
	for span in door_side:
		var x := HALF_WIDTH - 0.01
		quad(glazing, [Vector3(x, SILL, span.y), Vector3(x, SILL, span.x), Vector3(x, CANT, span.x), Vector3(x, CANT, span.y)], [Vector2.ZERO, Vector2.ZERO, Vector2.ZERO, Vector2.ZERO])
	var left_x := -(HALF_WIDTH - 0.01)
	quad(glazing, [Vector3(left_x, SILL, -SIDE_END), Vector3(left_x, SILL, SIDE_END), Vector3(left_x, CANT, SIDE_END), Vector3(left_x, CANT, -SIDE_END)], [Vector2.ZERO, Vector2.ZERO, Vector2.ZERO, Vector2.ZERO])
	for corner in [[1.0, -1.0], [-1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]:
		var outer := Vector3(corner[0] * HALF_WIDTH, 0, corner[1] * SIDE_END)
		var inner := Vector3(corner[0] * NOSE_HALF_WIDTH, 0, corner[1] * NOSE)
		var pair := [outer, inner] if corner[0] * corner[1] < 0 else [inner, outer]
		quad(glazing, [pair[0] + Vector3.UP * SILL, pair[1] + Vector3.UP * SILL, pair[1] + Vector3.UP * CANT, pair[0] + Vector3.UP * CANT], [Vector2.ZERO, Vector2.ZERO, Vector2.ZERO, Vector2.ZERO])
	var w := NOSE_HALF_WIDTH
	quad(glazing, [Vector3(w, FRONT_MASK_TOP, -NOSE), Vector3(-w, FRONT_MASK_TOP, -NOSE), Vector3(-w, ROOF, -NOSE), Vector3(w, ROOF, -NOSE)], [Vector2.ZERO, Vector2.ZERO, Vector2.ZERO, Vector2.ZERO])
	quad(glazing, [Vector3(-w, REAR_PANEL_TOP, NOSE), Vector3(w, REAR_PANEL_TOP, NOSE), Vector3(w, REAR_CAP_BOTTOM, NOSE), Vector3(-w, REAR_CAP_BOTTOM, NOSE)], [Vector2.ZERO, Vector2.ZERO, Vector2.ZERO, Vector2.ZERO])
	glazing.generate_normals()
	add_mesh(parent, glazing.commit(), glass)

	# Back faces only, behind the driver, so open doors show a cabin while the cab camera still sees out.
	var cabin := BoxMesh.new()
	cabin.size = Vector3(2.0 * HALF_WIDTH - 0.06, ROOF - 0.42, SIDE_END - 0.05 + CABIN_FRONT)
	var cabin_instance := add_mesh(parent, cabin, interior)
	cabin_instance.position = Vector3(0, 0.36 + cabin.size.y / 2.0, (SIDE_END - 0.05 - CABIN_FRONT) / 2.0)
	var cab := BoxMesh.new()
	cab.size = Vector3(cabin.size.x, SILL - 0.36, SIDE_END - CABIN_FRONT)
	var cab_instance := add_mesh(parent, cab, interior)
	cab_instance.position = Vector3(0, 0.36 + cab.size.y / 2.0, -(SIDE_END + CABIN_FRONT) / 2.0)

func add_mesh(parent: Node3D, mesh: Mesh, paint: Material) -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.material_override = paint
	parent.add_child(instance)
	return instance

# Textured wheel cover on the outer face of a wheel, in the wheel's own frame (axle along x).
func wheel_cover(parent: Node3D, outward: float, radius: float, offset: float) -> void:
	var tool := SurfaceTool.new()
	tool.begin(Mesh.PRIMITIVE_TRIANGLES)
	var segments := 24
	for index in range(segments):
		var angles := [TAU * index / segments, TAU * (index + 1) / segments]
		if outward < 0:
			angles.reverse()
		tool.set_uv(uv(WHEEL_COVER_CENTRE))
		tool.add_vertex(Vector3(outward * offset, 0, 0))
		for angle in [angles[0], angles[1]]:
			var direction := Vector2(cos(angle), sin(angle))
			tool.set_uv(uv(WHEEL_COVER_CENTRE + direction * WHEEL_COVER_RADIUS))
			tool.add_vertex(Vector3(outward * offset, direction.y * radius, direction.x * radius))
	tool.generate_normals()
	var paint := StandardMaterial3D.new()
	paint.albedo_texture = textures.repaint_body
	paint.roughness = 0.45
	paint.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
	add_mesh(parent, tool.commit(), paint)

func plate(parent: Node3D, position: Vector3, facing_front: bool) -> void:
	var plate_texture := texture("repaint_rejka")
	if plate_texture == null:
		return
	var instance := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = Vector2(0.52, 0.114)
	instance.mesh = mesh
	var paint := StandardMaterial3D.new()
	paint.albedo_texture = plate_texture
	paint.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
	instance.material_override = paint
	parent.add_child(instance)
	instance.position = position
	instance.rotation.y = PI if facing_front else 0.0
