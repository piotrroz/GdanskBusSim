extends Node3D

const Bus = preload("res://scripts/bus.gd")
const City = preload("res://scripts/world.gd")
const Hud = preload("res://scripts/hud.gd")
var data: Dictionary
var city: Node3D
var bus: CharacterBody3D
var hud: Control
var direction_index := 0
var next_stop := 0
var served := 0
var passengers := 0
var dwell := 0.0
var elapsed := 0.0
var finished := false
var paused := false
var status := ""
var photo_mode := false

func _ready() -> void:
	configure_input()
	data = JSON.parse_string(FileAccess.get_file_as_string("res://data/route_108.json"))
	var environment := WorldEnvironment.new()
	var settings := Environment.new()
	settings.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var sky_material := ProceduralSkyMaterial.new()
	sky_material.sky_top_color = Color("72aab7")
	sky_material.sky_horizon_color = Color("dce2d6")
	sky_material.ground_bottom_color = Color("7c9574")
	sky_material.ground_horizon_color = Color("dce2d6")
	sky.sky_material = sky_material
	settings.sky = sky
	settings.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	settings.ambient_light_color = Color("d4dfdf")
	settings.ambient_light_energy = 0.35
	settings.ambient_light_sky_contribution = 0.0
	settings.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	settings.fog_enabled = true
	settings.fog_light_color = Color("cfd9d6")
	settings.fog_density = 0.0012
	settings.fog_sky_affect = 0.0
	environment.environment = settings
	add_child(environment)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-42, -32, 0)
	sun.light_color = Color("fff1d7")
	sun.light_energy = 0.75
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 200
	add_child(sun)
	load_direction(0)
	var layer := CanvasLayer.new()
	add_child(layer)
	hud = Hud.new()
	hud.game = self
	layer.add_child(hud)

func configure_input() -> void:
	var bindings := {"accelerate": [KEY_W, KEY_UP], "brake": [KEY_S, KEY_DOWN], "left": [KEY_A, KEY_LEFT], "right": [KEY_D, KEY_RIGHT], "handbrake": [KEY_SPACE], "doors": [KEY_E], "camera": [KEY_C], "reset_bus": [KEY_R], "reverse": [KEY_Q], "direction": [KEY_TAB], "pause_game": [KEY_ESCAPE], "orthophoto": [KEY_O]}
	for action in bindings:
		if not InputMap.has_action(action):
			InputMap.add_action(action)
		for key in bindings[action]:
			var event := InputEventKey.new()
			event.physical_keycode = key
			InputMap.action_add_event(action, event)

func load_direction(index: int) -> void:
	if is_instance_valid(city):
		remove_child(city)
		city.queue_free()
		remove_child(bus)
		bus.queue_free()
	direction_index = index
	city = City.new()
	add_child(city)
	city.build(data.directions[index])
	city.set_photo_mode(photo_mode)
	bus = Bus.new()
	add_child(bus)
	bus.set_destination("108", str(city.route.headsign))
	next_stop = 0
	served = 0
	passengers = 0
	dwell = 0
	elapsed = 0
	finished = false
	reset_bus()

func reset_bus() -> void:
	var index := mini(next_stop, city.stop_positions.size() - 1)
	var match_point: Dictionary = city.nearest(city.stop_positions[index], maxf(0, city.stop_distances[index] - 20))
	var tangent: Vector3 = match_point.tangent
	bus.position = match_point.point + tangent.cross(Vector3.UP) * 2.5 + Vector3.UP * 0.12
	bus.position.y = city.surface_height(bus.position) + 0.6
	bus.rotation = Vector3(0, atan2(-tangent.x, -tangent.z), 0)
	bus.reset_motion()
	bus.doors_open = false
	bus.reverse = false
	dwell = 0
	bus.update_camera()

func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("pause_game"):
		paused = not paused
		bus.enabled = not paused
	if paused:
		return
	if event.is_action_pressed("doors"):
		bus.toggle_doors()
	if event.is_action_pressed("camera"):
		bus.cab_view = not bus.cab_view
		bus.update_camera()
	if event.is_action_pressed("orthophoto"):
		photo_mode = not photo_mode
		city.set_photo_mode(photo_mode)
	if event.is_action_pressed("reset_bus"):
		reset_bus()
	if event.is_action_pressed("reverse") and absf(bus.speed) < 0.15:
		bus.reverse = not bus.reverse
	if event.is_action_pressed("direction") and absf(bus.speed) < 0.15:
		load_direction(1 - direction_index)

func _process(delta: float) -> void:
	if not paused:
		elapsed += delta
		update_service(delta)
		bus.payload_kg = passengers * bus.PASSENGER_MASS
	if is_instance_valid(hud):
		hud.queue_redraw()

func update_service(delta: float) -> void:
	if finished:
		status = "SERVICE COMPLETE"
		return
	var distance: float = bus.position.distance_to(city.stop_positions[next_stop])
	if distance < 19 and absf(bus.speed) < 0.15:
		if bus.doors_open:
			dwell += delta
			status = "BOARDING  %ds" % ceili(maxf(0, 5 - dwell))
			if dwell >= 5:
				passengers = clampi(passengers + 3 + (next_stop * 7) % 9 - (next_stop * 3) % 7, 0, 75)
				served += 1
				next_stop += 1
				dwell = 0
				if next_stop >= city.stop_positions.size():
					finished = true
					passengers = 0
		else:
			dwell = 0
			status = "AT STOP / DOORS CLOSED"
	else:
		dwell = 0
		status = "DOORS OPEN" if bus.doors_open else "IN SERVICE"