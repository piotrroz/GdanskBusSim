extends SceneTree

# Run with: --headless --fixed-fps 60 --script tests/bus_dynamics.gd
const Bus = preload("res://scripts/bus.gd")
const BusLivery = preload("res://scripts/bus_livery.gd")

var failures := 0
var root_node: Node3D
var bus

func check(condition: bool, message: String) -> void:
	if not condition:
		failures += 1
		push_error("BUS DYNAMICS FAIL: " + message)

func _initialize() -> void:
	run.call_deferred()

func slab(size: Vector3, at: Vector3, pitch := 0.0) -> void:
	var floor_body := StaticBody3D.new()
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	shape.shape = box
	floor_body.add_child(shape)
	root_node.add_child(floor_body)
	floor_body.position = at
	floor_body.rotation.x = pitch

func spawn(at: Vector3, yaw := 0.0) -> void:
	if bus:
		bus.free()
	bus = Bus.new()
	bus.player_controlled = false
	root_node.add_child(bus)
	bus.position = at
	bus.rotation.y = yaw
	bus.reset_motion()

func controls(throttle: float, brake: float, steer := 0.0) -> void:
	bus.throttle_input = throttle
	bus.brake_input = brake
	bus.steer_input = steer

func frames(count: int) -> void:
	for index in range(count):
		await physics_frame

func check_repaint() -> void:
	var sample := "[item]\nPKS #1\nrepaint_body\nPKS #1\\body.png\n\n[setvar]\nEV_kaganiec\n3\n\n[setvar]\nbroken\nx\n"
	var parsed: Dictionary = BusLivery.parse_cti(sample)
	check(parsed.name == "PKS #1" and parsed.items.repaint_body == "PKS #1/body.png", "CTI items resolve with forward slashes")
	check(parsed.settings.get("EV_kaganiec") == 3 and not parsed.settings.has("broken"), "CTI integer settings parse, malformed ones are skipped")
	var plain = Bus.new()
	plain.player_controlled = false
	plain.repaint_path = "res://Vehicles/missing.cti"
	root_node.add_child(plain)
	check(plain.top_speed == Bus.DEFAULT_TOP_SPEED and plain.door_panels.size() == 3 and plain.rear_display != null, "Missing repaint falls back to the procedural body")
	plain.free()
	var path := BusLivery.find_default()
	if path.is_empty():
		print("Repaint: none installed, procedural body")
		return
	var livery = BusLivery.new()
	check(livery.load_repaint(path), "Installed repaint must provide a body texture: " + path)
	print("Repaint: %s, %d texture slots, %.0f km/h, %.0f km range" % [livery.name, livery.texture_paths.size(), livery.top_speed_kmh(), livery.range_km()])
	spawn(Vector3(0, 0.3, 0))
	check(is_equal_approx(bus.top_speed, livery.top_speed_kmh() / 3.6), "Repaint speed limiter drives the governor")
	check(bus.door_panels.size() == BusLivery.DOORS.size(), "Doors follow the repaint's openings")

func run() -> void:
	root_node = Node3D.new()
	root.add_child(root_node)
	slab(Vector3(4000, 1, 4000), Vector3(0, -0.5, 0))
	check_repaint()
	# 8% ramp rising towards -z, far from the flat test area.
	var grade := atan(0.08)
	slab(Vector3(40, 1, 400), Vector3(3000, 0, 0), grade)

	spawn(Vector3(0, 0.3, 1500))
	await frames(30)
	controls(1, 0)
	var time := 0.0
	while bus.speed < 50.0 / 3.6 and time < 40:
		await physics_frame
		time += 1.0 / 60.0
	print("0-50 km/h: %.1f s" % time)
	check(time > 8.0 and time < 16.0, "0-50 km/h should take 8-16 s, took %.1f" % time)
	var top := 0.0
	for index in range(60 * 60):
		await physics_frame
		top = maxf(top, bus.speed)
	print("Top speed: %.1f km/h (governor %.0f)" % [top * 3.6, bus.top_speed * 3.6])
	check(top > bus.top_speed - 0.8 and top < bus.top_speed + 0.2, "Bus must reach and respect its governed top speed")

	spawn(Vector3(0, 0.3, 1500))
	await frames(30)
	controls(1, 0)
	while bus.speed < 50.0 / 3.6:
		await physics_frame
	controls(0, 1)
	var start: Vector3 = bus.global_position
	while bus.speed > 0.0:
		await physics_frame
	var stopping := start.distance_to(bus.global_position)
	print("Full brake from 50 km/h: %.1f m" % stopping)
	check(stopping > 13.0 and stopping < 24.0, "Braking distance from 50 km/h, got %.1f m" % stopping)
	check(bus.hold, "Braking to a stop engages hold")

	spawn(Vector3(0, 0.3, 0))
	await frames(30)
	controls(0.3, 0, 1.0)
	while bus.speed < 2.5:
		await physics_frame
	controls(0, 0, 1.0)
	await frames(90)
	var points: Array[Vector3] = []
	var max_slip := 0.0
	for sample in range(3):
		var before: Vector3 = bus.rear_axle_position()
		await physics_frame
		var after: Vector3 = bus.rear_axle_position()
		var right: Vector3 = bus.yaw_basis() * Vector3.RIGHT
		max_slip = maxf(max_slip, absf((after - before).dot(right)) / maxf((after - before).length(), 0.001))
		points.append(after)
		await frames(40)
	var a := points[0].distance_to(points[1])
	var b := points[1].distance_to(points[2])
	var c := points[2].distance_to(points[0])
	var area := (points[1] - points[0]).cross(points[2] - points[0]).length() / 2.0
	var radius := a * b * c / (4.0 * area)
	var expected := Bus.WHEELBASE / tan(bus.steering)
	print("Full lock rear-axle radius %.2f m (expected %.2f), slip ratio %.4f" % [radius, expected, max_slip])
	check(absf(radius - expected) / expected < 0.05, "Rear axle follows L / tan(delta)")
	check(max_slip < 0.03, "Rear axle must not slide sideways")

	spawn(Vector3(0, 0.3, 1000))
	await frames(30)
	controls(1, 0)
	while bus.speed < 15.0:
		await physics_frame
	controls(0.3, 0, 1.0)
	var peak_lateral := 0.0
	for index in range(180):
		await physics_frame
		peak_lateral = maxf(peak_lateral, absf(bus.lateral_acceleration))
	print("Peak lateral acceleration at %.0f km/h: %.2f m/s2" % [bus.speed * 3.6, peak_lateral])
	check(peak_lateral <= Bus.GRIP_LIMIT + 0.05, "Lateral acceleration capped by grip")

	# Ramp: surface at x=3000 rises 8% towards -z; place the bus facing uphill.
	var ramp_height := 0.5 / cos(grade)
	spawn(Vector3(3000, ramp_height + 0.3, 0), 0.0)
	controls(0, 0)
	await frames(90)
	var parked: Vector3 = bus.global_position
	await frames(120)
	var creep := parked.distance_to(bus.global_position)
	print("Hold on 8%%: moved %.3f m, pitch %.3f rad" % [creep, bus.rotation.x])
	check(creep < 0.05, "Hold keeps the bus on an 8% grade")
	check(absf(bus.rotation.x - grade) < 0.01, "Body pitch follows the ground")
	controls(1, 0)
	var lowest: float = -bus.global_position.z
	for index in range(240):
		await physics_frame
		lowest = minf(lowest, -bus.global_position.z)
	print("Hill start: rollback %.3f m, then %.1f m uphill" % [-parked.z - lowest, -bus.global_position.z + parked.z])
	check(-parked.z - lowest < 0.05, "Hill-hold prevents rollback on start")
	check(-bus.global_position.z + parked.z > 2.0, "Bus climbs an 8% grade")
	controls(0, 0)
	bus.hold = false
	bus.speed = 0
	await frames(1)
	bus.hold = false
	var released: Vector3 = bus.global_position
	await frames(180)
	print("Released on 8%%: rolled %.1f m back, speed %.2f m/s" % [bus.global_position.z - released.z, bus.speed])
	check(bus.global_position.z - released.z > 1.0, "Without brakes the bus rolls back downhill")

	print("BUS DYNAMICS: %s" % ("PASS" if failures == 0 else "%d failures" % failures))
	quit(0 if failures == 0 else 1)
