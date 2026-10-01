extends CharacterBody3D

const BusLivery = preload("res://scripts/bus_livery.gd")

# Approximate Solaris Urbino 12 electric: published-class dimensions, illustrative drivetrain figures.
const G := 9.81
const MASS_EMPTY := 13500.0
const PASSENGER_MASS := 70.0
const ROTATING_MASS_FACTOR := 1.05
const WHEELBASE := 5.9
const FRONT_AXLE := -3.3
const REAR_AXLE := 2.6
const HALF_TRACK := 1.05
const WHEEL_RADIUS := 0.52
const MAX_WHEEL_ANGLE := 0.66
const PEAK_FORCE := 20000.0
const PEAK_POWER := 160000.0
const REVERSE_SPEED := 10.0 / 3.6
const MAX_BRAKE := 6.5
const COAST_REGEN := 0.3
const ROLLING := 0.008
const DRAG_AREA := 0.65 * 8.0
const AIR_DENSITY := 1.2
const GRIP_LIMIT := 5.5
const COMFORT_LATERAL := 3.2
const REGEN_FORCE := 16000.0
const DEFAULT_TOP_SPEED := 70.0 / 3.6
const DEFAULT_BATTERY_KWH := 400.0
# Illustrative average draw used to turn the repaint's nominal range into a capacity.
const CONSUMPTION_KWH_PER_KM := 1.2
const AUX_POWER := 8000.0
const CAB_EYE := Vector3(-0.67, 2.34, -5.63)
const CHASE_OFFSET := Vector3(7.5, 6.5, 14)

var speed := 0.0
var steering := 0.0
var doors_open := false
var reverse := false
var battery := 100.0
var distance_driven := 0.0
var door_panels: Array[Node3D] = []
var front_wheels: Array[Node3D] = []
var wheels: Array[Node3D] = []
var wheel_spinners: Array[Node3D] = []
var destination: Label3D
var camera: Camera3D
var cab_view := false
var enabled := true
var door_blend := 0.0
var player_controlled := true
var throttle_input := 0.0
var brake_input := 0.0
var steer_input := 0.0
var handbrake_input := false
var throttle := 0.0
var brake := 0.0
var hold := true
var payload_kg := 0.0
var power_kw := 0.0
var acceleration := 0.0
var yaw_rate := 0.0
var lateral_acceleration := 0.0
var body: Node3D
var steering_wheel: MeshInstance3D
var tail_light: StandardMaterial3D
var wheel_spin := 0.0
var kneel := 0.0
var suspension := Vector3.ZERO
var suspension_velocity := Vector3.ZERO
var repaint_path := ""
var livery: BusLivery
var top_speed := DEFAULT_TOP_SPEED
var battery_kwh := DEFAULT_BATTERY_KWH
var door_leaves: Array = []
var rear_display: Label3D
var side_displays: Array[Label3D] = []

static func tractive_force(pedal: float, velocity_along: float, in_reverse: bool, forward_limit := DEFAULT_TOP_SPEED) -> float:
	var direction := -1.0 if in_reverse else 1.0
	var limit := REVERSE_SPEED if in_reverse else forward_limit
	var governor := clampf((limit - velocity_along * direction) / 1.0, 0.0, 1.0)
	return direction * pedal * governor * minf(PEAK_FORCE, PEAK_POWER / maxf(absf(velocity_along), 0.1))

static func resistance_force(velocity_along: float, mass: float) -> float:
	return ROLLING * mass * G + 0.5 * AIR_DENSITY * DRAG_AREA * velocity_along * velocity_along

static func effective_wheel_angle(angle: float, velocity_along: float) -> float:
	# Beyond the tyre grip the bus runs wide instead of following the wheel angle.
	var limit := atan(GRIP_LIMIT * WHEELBASE / maxf(velocity_along * velocity_along, 0.01))
	return clampf(angle, -limit, limit)

func material(color: String, glow := false) -> StandardMaterial3D:
	var result := StandardMaterial3D.new()
	result.albedo_color = Color(color)
	result.roughness = 0.65
	if glow:
		result.emission_enabled = true
		result.emission = Color(color)
		result.emission_energy_multiplier = 1.8
	return result

func box(size: Vector3, at: Vector3, paint: Material, parent: Node3D = null) -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	instance.mesh = mesh
	instance.material_override = paint
	(parent if parent else body).add_child(instance)
	instance.position = at
	return instance

func _ready() -> void:
	floor_snap_length = 1.0
	floor_constant_speed = true
	collision_mask |= 2
	body = Node3D.new()
	body.name = "Body"
	add_child(body)
	var red := material("b32132")
	var ivory := material("e6e3d6")
	var glass := material("162b30")
	glass.metallic = 0.35
	glass.roughness = 0.12
	var rubber := material("161a1c")
	var metal := material("97a8ac")
	var frame := material("2d3236")
	livery = BusLivery.new()
	var textured: bool = livery.load_repaint(repaint_path if repaint_path != "" else BusLivery.find_default())
	if textured:
		top_speed = livery.top_speed_kmh() / 3.6
		battery_kwh = livery.range_km() * CONSUMPTION_KWH_PER_KM
		var cabin := material("2a3034")
		cabin.cull_mode = BaseMaterial3D.CULL_FRONT
		livery.build_shell(body, glass, cabin)
		livery.plate(body, Vector3(0, 0.515, -6.006), true)
		livery.plate(body, Vector3(0, 0.57, 6.006), false)
	else:
		build_plain_shell(red, ivory, glass, metal)
	box(Vector3(2.0, 0.36, 4.3), Vector3(0, 3.38, 1.8), ivory)
	box(Vector3(1.6, 0.24, 1.6), Vector3(0, 3.33, -2.5), ivory if textured else metal)
	for side in [-1.0, 1.0]:
		box(Vector3(0.12, 0.09, 0.85), Vector3(side * 1.48, 2.7, -5.2), rubber)
		box(Vector3(0.20, 0.5, 0.30), Vector3(side * 1.58, 2.48, -4.9), rubber)
		for longitudinal in [FRONT_AXLE, REAR_AXLE]:
			# Wheels hang from the unsprung frame so the body can pitch and roll above them.
			var pivot := Node3D.new()
			add_child(pivot)
			pivot.position = Vector3(side * HALF_TRACK, WHEEL_RADIUS, longitudinal)
			wheels.append(pivot)
			if longitudinal < 0:
				front_wheels.append(pivot)
			var spinner := Node3D.new()
			pivot.add_child(spinner)
			wheel_spinners.append(spinner)
			var tire := MeshInstance3D.new()
			var cylinder := CylinderMesh.new()
			cylinder.top_radius = WHEEL_RADIUS
			cylinder.bottom_radius = WHEEL_RADIUS
			cylinder.height = 0.28
			cylinder.radial_segments = 24
			tire.mesh = cylinder
			tire.material_override = rubber
			tire.rotation.z = PI / 2
			spinner.add_child(tire)
			if textured:
				livery.wheel_cover(spinner, side, 0.36, 0.142)
			else:
				var hub := MeshInstance3D.new()
				var hub_mesh := CylinderMesh.new()
				hub_mesh.top_radius = 0.29
				hub_mesh.bottom_radius = 0.29
				hub_mesh.height = 0.30
				hub.mesh = hub_mesh
				hub.material_override = metal
				hub.rotation.z = PI / 2
				spinner.add_child(hub)
				for spoke in range(3):
					var bar := box(Vector3(0.32, 0.07, 0.5), Vector3.ZERO, rubber, spinner)
					bar.rotation.x = spoke * PI / 3
	var openings: Array = BusLivery.DOORS if textured else [Vector2(-5.275, -4.125), Vector2(-0.975, 0.175), Vector2(3.625, 4.775)]
	for opening in openings:
		var door := Node3D.new()
		body.add_child(door)
		door.position = Vector3(BusLivery.HALF_WIDTH + 0.02, 0, (opening.x + opening.y) / 2.0)
		door_panels.append(door)
		var leaf_width: float = (opening.y - opening.x) / 2.0 - 0.02
		for direction in [-1.0, 1.0]:
			# Plug doors: each leaf swings out, then slides along the body.
			var leaf := Node3D.new()
			door.add_child(leaf)
			leaf.position.z = direction * (leaf_width / 2.0 + 0.01)
			door_leaves.append([leaf, direction, leaf_width, minf(leaf_width * 0.85, (opening.x + 5.9) if direction < 0 else (5.9 - opening.y))])
			box(Vector3(0.05, 2.36, leaf_width), Vector3(0, 1.53, 0), frame, leaf)
			box(Vector3(0.06, 1.92, leaf_width - 0.12), Vector3(0, 1.68, 0), glass, leaf)
			box(Vector3(0.06, 0.28, leaf_width - 0.06), Vector3(0, 0.5, 0), red, leaf)
	var display_backing := Vector3(2.0, 0.4, 0.04) if textured else Vector3(2.30, 0.48, 0.055)
	var display_height := 3.0 if textured else 2.77
	box(display_backing, Vector3(0, display_height, -5.97), rubber)
	destination = display_label(48, Vector3(0, display_height, -6.01), PI)
	rear_display = display_label(64 if livery.setting("wyswietlacz_tyl", 1) == 1 else 44, Vector3(0, 2.94 if textured else 2.75, 6.035), 0.0)
	box(Vector3(0.62, 0.22, 0.03), Vector3(0, 2.94 if textured else 2.75, 6.015), rubber)
	var side_sizes := [livery.setting("wyswietlacz_bok_drzwi", 0), livery.setting("wyswietlacz_bok_kier", 1)]
	for index in range(2):
		var side := 1.0 if index == 0 else -1.0
		if index == 1 and side_sizes[1] == 0:
			continue
		var wide := 1.1 if index == 0 and side_sizes[0] == 1 else 0.85
		box(Vector3(0.03, 0.2, wide), Vector3(side * 1.27, 2.5, -3.6), rubber)
		side_displays.append(display_label(26, Vector3(side * 1.29, 2.5, -3.6), side * PI / 2))
	tail_light = material("a0141a", true)
	var headlight := material("fff4d3", true)
	headlight.emission_energy_multiplier = 1.0
	for side in [-1.0, 1.0]:
		if textured:
			box(Vector3(0.34, 0.08, 0.03), Vector3(side * 0.86, 0.57, -6.015), headlight)
			box(Vector3(0.09, 0.5, 0.03), Vector3(side * 1.06, 1.2, 6.015), tail_light)
		else:
			box(Vector3(0.56, 0.17, 0.06), Vector3(side * 0.84, 1.12, -6.025), headlight)
			box(Vector3(0.14, 0.48, 0.06), Vector3(side * 1.04, 1.26, 6.025), tail_light)
	box(Vector3(2.0, 0.4, 0.70), Vector3(0, 1.48, -5.24), rubber)
	box(Vector3(0.54, 0.055, 0.34), Vector3(-0.65, 1.71, -5.08), metal)
	steering_wheel = MeshInstance3D.new()
	var torus := TorusMesh.new()
	torus.inner_radius = 0.18
	torus.outer_radius = 0.23
	steering_wheel.mesh = torus
	steering_wheel.material_override = rubber
	steering_wheel.position = Vector3(-0.68, 1.52, -4.77)
	steering_wheel.rotation.x = 0.35
	body.add_child(steering_wheel)
	box(Vector3(0.05, 0.03, 0.40), Vector3.ZERO, metal, steering_wheel)
	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = Vector3(2.5, 2.8, 11.9)
	collision.shape = shape
	collision.position.y = 1.48
	add_child(collision)
	camera = Camera3D.new()
	add_child(camera)
	camera.current = true
	camera.far = 2200
	update_camera()

func yaw_basis() -> Basis:
	return Basis(Vector3.UP, rotation.y)

func rear_axle_position() -> Vector3:
	return global_position + yaw_basis() * Vector3(0, 0, REAR_AXLE)

func update_camera() -> void:
	camera.top_level = not cab_view
	if cab_view:
		camera.transform = body.transform * Transform3D(Basis(), CAB_EYE)
		camera.fov = 78
	else:
		camera.global_position = global_position + yaw_basis() * CHASE_OFFSET
		camera.look_at(global_position + yaw_basis() * Vector3(0, 1.1, -4))
		camera.fov = 62

func follow_camera(delta: float) -> void:
	if cab_view:
		camera.transform = body.transform * Transform3D(Basis(), CAB_EYE)
		return
	var target := global_position + yaw_basis() * CHASE_OFFSET
	camera.global_position = camera.global_position.lerp(target, 1.0 - exp(-delta * 5.0))
	camera.look_at(global_position + yaw_basis() * Vector3(0, 1.1, -4))

func reset_motion() -> void:
	speed = 0
	steering = 0
	throttle = 0
	brake = 0
	hold = true
	acceleration = 0
	yaw_rate = 0
	lateral_acceleration = 0
	velocity = Vector3.ZERO
	suspension = Vector3.ZERO
	suspension_velocity = Vector3.ZERO

func toggle_doors() -> void:
	if absf(speed) < 0.15:
		doors_open = not doors_open
		if doors_open:
			speed = 0
			hold = true

func sample_ground(delta: float) -> void:
	var space := get_world_3d().direct_space_state
	var heights: Array[float] = []
	for corner in [Vector3(-HALF_TRACK, 0, FRONT_AXLE), Vector3(HALF_TRACK, 0, FRONT_AXLE), Vector3(-HALF_TRACK, 0, REAR_AXLE), Vector3(HALF_TRACK, 0, REAR_AXLE)]:
		var at: Vector3 = global_position + yaw_basis() * corner
		var query := PhysicsRayQueryParameters3D.create(at + Vector3.UP * 2.0, at - Vector3.UP * 2.0, 3)
		query.exclude = [get_rid()]
		var hit := space.intersect_ray(query)
		if hit.is_empty():
			return
		heights.append(hit.position.y)
	var pitch := atan2(heights[0] + heights[1] - heights[2] - heights[3], 2.0 * WHEELBASE)
	var roll := atan2(heights[1] + heights[3] - heights[0] - heights[2], 4.0 * HALF_TRACK)
	var blend := 1.0 - exp(-delta * 10.0)
	rotation.x = lerpf(rotation.x, pitch, blend)
	rotation.z = lerpf(rotation.z, roll, blend)

func _physics_process(delta: float) -> void:
	if not enabled:
		return
	if player_controlled:
		throttle_input = Input.get_action_strength("accelerate")
		brake_input = Input.get_action_strength("brake")
		steer_input = Input.get_axis("right", "left")
		handbrake_input = Input.is_action_pressed("handbrake")
	# Pedals and steering move at human rates, not instantly.
	throttle = move_toward(throttle, throttle_input, delta * (2.0 if throttle_input > throttle else 4.0))
	brake = move_toward(brake, brake_input, delta * (2.5 if brake_input > brake else 5.0))
	var moving := absf(speed)
	var steer_limit := minf(MAX_WHEEL_ANGLE, atan(COMFORT_LATERAL * WHEELBASE / maxf(moving * moving, 0.01)))
	var steer_rate := lerpf(0.8, 0.3, clampf(moving / 14.0, 0.0, 1.0)) * (1.6 if steer_input == 0 else 1.0)
	steering = move_toward(steering, steer_input * steer_limit, delta * steer_rate)
	sample_ground(delta)

	var mass := MASS_EMPTY + payload_kg
	var forward := -global_basis.z
	var grade_force := -mass * G * forward.y
	var interlock := doors_open or handbrake_input
	var drive_sign := -1.0 if reverse else 1.0
	var drive := 0.0 if interlock else tractive_force(throttle, speed, reverse, top_speed)
	var previous_speed := speed
	var brake_force := brake * MAX_BRAKE * mass
	var coast_force := 0.0 if throttle > 0.02 else COAST_REGEN * mass * clampf(moving / 3.0, 0.0, 1.0)
	if moving < 0.1 and (brake > 0.2 or interlock):
		hold = true
	if hold and throttle_input > 0.05 and not interlock and (drive + grade_force) * drive_sign > 0:
		hold = false  # Hill-hold releases only once traction exceeds the slope.
	if hold:
		speed = 0
	else:
		var resist := resistance_force(speed, mass) + brake_force + coast_force
		var propel := drive + grade_force
		if moving < 0.05 and absf(propel) <= resist:
			speed = 0
		else:
			var direction := signf(speed) if moving >= 0.05 else signf(propel)
			var next_speed := speed + (propel - direction * resist) / (mass * ROTATING_MASS_FACTOR) * delta
			speed = 0.0 if next_speed * direction < 0 else next_speed

	var wheel_angle := effective_wheel_angle(steering, speed)
	yaw_rate = speed * tan(wheel_angle) / WHEELBASE
	lateral_acceleration = speed * yaw_rate
	rotation.y += yaw_rate * delta
	# Kinematic bicycle about the rear axle: the rear wheels never slip sideways, the front overhang swings out.
	var flat_forward := yaw_basis() * Vector3.FORWARD
	var left := Vector3.UP.cross(flat_forward)
	var fall_speed := velocity.y
	velocity = flat_forward * speed + left * yaw_rate * REAR_AXLE
	velocity.y = -1.0 if is_on_floor() else minf(0.0, fall_speed) - G * delta
	move_and_slide()
	for index in range(get_slide_collision_count()):
		if absf(get_slide_collision(index).get_normal().y) < 0.5:
			var along := get_real_velocity().dot(flat_forward)
			speed = along if along * speed > 0 and absf(along) < absf(speed) else 0.0
			break
	acceleration = lerpf(acceleration, (speed - previous_speed) / delta, 1.0 - exp(-delta * 8.0))

	var wheel_power := drive * speed
	var regen := minf(brake_force + coast_force, REGEN_FORCE) * moving if not hold else 0.0
	power_kw = (maxf(wheel_power, 0.0) / 0.9 - minf(regen, PEAK_POWER) * 0.7) / 1000.0
	battery = clampf(battery - (power_kw * 1000.0 + AUX_POWER) * delta / (battery_kwh * 3.6e6) * 100.0, 0.0, 100.0)
	distance_driven += absf(speed) * delta
	animate(delta)
	follow_camera(delta)

func animate(delta: float) -> void:
	wheel_spin = fmod(wheel_spin + speed / WHEEL_RADIUS * delta, TAU)
	for spinner in wheel_spinners:
		spinner.rotation.x = -wheel_spin
	# Ackermann: the inner front wheel turns more than the outer one.
	for index in range(front_wheels.size()):
		var side := -1.0 if index == 0 else 1.0
		if absf(steering) < 0.001:
			front_wheels[index].rotation.y = steering
		else:
			front_wheels[index].rotation.y = atan(WHEELBASE / (WHEELBASE / tan(steering) + side * HALF_TRACK))
	steering_wheel.basis = Basis(Vector3.RIGHT, 0.35) * Basis(Vector3.UP, steering * 16.0)
	tail_light.emission_energy_multiplier = 0.6 + 1.6 * maxf(brake, 1.0 if hold else 0.0)
	# Damped body motion: squat, dive, outward roll and door-side kneeling at stops.
	kneel = move_toward(kneel, 1.0 if doors_open else 0.0, delta * 0.6)
	var target := Vector3(clampf(acceleration * 0.005, -0.03, 0.02), -0.03 * kneel,
		clampf(-lateral_acceleration * 0.009, -0.05, 0.05) - 0.035 * kneel)
	suspension_velocity += ((target - suspension) * 40.0 - suspension_velocity * 5.0) * delta
	suspension += suspension_velocity * delta
	body.transform = Transform3D(Basis.from_euler(Vector3(suspension.x, 0, suspension.z)), Vector3(0, suspension.y, 0))
	door_blend = move_toward(door_blend, 1.0 if doors_open else 0.0, delta * 0.4)
	var swing := clampf(door_blend / 0.3, 0.0, 1.0)
	var slide := clampf((door_blend - 0.3) / 0.7, 0.0, 1.0)
	for entry in door_leaves:
		var leaf: Node3D = entry[0]
		leaf.position.x = smoothstep(0.0, 1.0, swing) * 0.1
		leaf.position.z = entry[1] * (entry[2] / 2.0 + 0.01 + smoothstep(0.0, 1.0, slide) * entry[3])

func display_label(size: int, at: Vector3, yaw: float) -> Label3D:
	var label := Label3D.new()
	label.font_size = size
	label.pixel_size = 0.004
	label.modulate = Color("ffb547")
	label.outline_size = 0
	label.position = at
	label.rotation.y = yaw
	body.add_child(label)
	return label

func set_destination(line: String, headsign: String) -> void:
	destination.text = line + "  " + headsign
	rear_display.text = line
	for display in side_displays:
		display.text = line + "  " + headsign

func build_plain_shell(red: Material, ivory: Material, glass: Material, metal: Material) -> void:
	box(Vector3(2.55, 1.05, 12.0), Vector3(0, 1.12, 0), red)
	box(Vector3(2.53, 1.37, 11.85), Vector3(0, 2.28, 0), glass)
	box(Vector3(2.56, 0.28, 12.0), Vector3(0, 3.07, 0), ivory)
	box(Vector3(2.57, 0.13, 11.9), Vector3(0, 1.68, 0), ivory)
	for side in [-1.0, 1.0]:
		for longitudinal in [-5.65, -3.6, -1.5, 0.6, 2.7, 4.8, 5.7]:
			box(Vector3(0.045, 1.32, 0.10), Vector3(side * 1.28, 2.3, longitudinal), ivory)
	box(Vector3(0.55, 0.14, 0.03), Vector3(0, 0.82, -6.035), ivory)
	box(Vector3(2.10, 0.23, 0.055), Vector3(0, 0.65, -6.02), material("161a1c"))
	for vent in range(7):
		box(Vector3(1.24, 0.03, 0.05), Vector3(0, 1.08 + vent * 0.10, 6.04), metal)