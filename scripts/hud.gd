extends Control

var game: Node3D
var font: Font
const INK := Color("15282b")
const PAPER := Color("f3f3e9")
const MUTED := Color("a6b9b6")
const GOLD := Color("edc36d")
const MINT := Color("9cd9bd")

func _ready() -> void:
	font = ThemeDB.fallback_font
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func text(value: String, at: Vector2, size: int, color := PAPER) -> void:
	draw_string(font, at, value, HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)

func fitted(value: String, at: Vector2, size: int, width: float, color := PAPER) -> void:
	var actual := size
	while font.get_string_size(value, HORIZONTAL_ALIGNMENT_LEFT, -1, actual).x > width and actual > 11:
		actual -= 1
	text(value, at, actual, color)

func _draw() -> void:
	if not is_instance_valid(game.bus):
		return
	var viewport := get_viewport_rect().size
	var scale_factor := minf(viewport.x / 1280.0, viewport.y / 720.0)
	draw_set_transform(Vector2((viewport.x - 1280 * scale_factor) / 2, (viewport.y - 720 * scale_factor) / 2), 0, Vector2.ONE * scale_factor)
	draw_style_box(panel(Color("15282be8")), Rect2(26, 24, 560, 100))
	draw_rect(Rect2(26, 24, 7, 100), Color("c74045"))
	text("108", Vector2(48, 77), 44, GOLD)
	text("GDANSK / ELECTRIC", Vector2(158, 48), 12, MUTED)
	fitted(str(game.city.route.headsign), Vector2(157, 80), 26, 407)
	text("ZTM   /   " + game.status, Vector2(158, 105), 12, MINT)
	draw_style_box(panel(Color("15282be8")), Rect2(900, 24, 354, 100))
	text("NEXT STOP", Vector2(920, 49), 12, MUTED)
	var index := mini(game.next_stop, game.city.route.stops.size() - 1)
	fitted("Terminus" if game.finished else str(game.city.route.stops[index].name), Vector2(920, 78), 23, 314)
	var match_point: Dictionary = game.city.nearest(game.bus.position)
	var remaining := maxf(0, float(game.city.stop_distances[index]) - float(match_point.distance))
	text("%03d m   /   %02d OF %02d STOPS" % [remaining, game.served, game.city.route.stops.size()], Vector2(920, 105), 12, GOLD)
	draw_style_box(panel(Color("15282bf2")), Rect2(26, 565, 365, 127))
	text("%02d" % roundi(absf(game.bus.speed) * 3.6), Vector2(46, 638), 58)
	text("km/h", Vector2(135, 635), 17, MUTED)
	text("R" if game.bus.reverse else "D", Vector2(203, 633), 33, GOLD)
	if game.bus.hold:
		text("HOLD", Vector2(200, 596), 12, Color("ff7a70"))
	draw_circle(Vector2(315, 615), 25, PAPER)
	text("50", Vector2(300, 623), 22, INK)
	draw_arc(Vector2(315, 615), 25, 0, TAU, 48, Color("df5b57"), 5, true)
	text("BATTERY  %.1f%%   %+d kW" % [game.bus.battery, roundi(game.bus.power_kw)], Vector2(46, 671), 12, MINT)
	text("ON BOARD  %02d" % game.passengers, Vector2(222, 671), 12, MUTED)
	draw_line(Vector2(46, 650), Vector2(370, 650), Color("49605d"))
	draw_style_box(panel(Color("15282bed")), Rect2(1014, 440, 240, 252))
	text("ROUTE / 108", Vector2(1030, 465), 12, GOLD)
	draw_map(Rect2(1030, 478, 208, 194))
	fitted("ZTM / CC BY | (c) OpenStreetMap / ODbL | GUGiK NMT + LiDAR 2018 / ortho 2021", Vector2(410, 691), 11, 585, INK)
	if game.dwell > 0:
		draw_rect(Rect2(465, 626, 340, 42), INK)
		draw_rect(Rect2(465, 665, 340 * game.dwell / 5.0, 3), MINT)
		text(game.status, Vector2(489, 654), 17, MINT)
	if game.paused or game.finished:
		draw_rect(Rect2(0, 0, 1280, 720), Color(0.04, 0.09, 0.10, 0.72))
		text("SERVICE COMPLETE" if game.finished else "PAUSED", Vector2(410, 295), 36, GOLD)
		text("108 / " + str(game.city.route.headsign), Vector2(410, 338), 23)
		text("%d stops served   /   %d passengers on board" % [game.served, game.passengers], Vector2(410, 374), 17, MUTED)

func panel(color: Color) -> StyleBoxFlat:
	var style := StyleBoxFlat.new()
	style.bg_color = color
	style.corner_radius_top_left = 4
	style.corner_radius_top_right = 4
	style.corner_radius_bottom_left = 4
	style.corner_radius_bottom_right = 4
	return style

func draw_map(rect: Rect2) -> void:
	var bounds := Rect2(Vector2(game.city.points[0].x, game.city.points[0].z), Vector2.ONE)
	for point in game.city.points:
		bounds = bounds.expand(Vector2(point.x, point.z))
	var factor := minf((rect.size.x - 20) / bounds.size.x, (rect.size.y - 20) / bounds.size.y)
	var center := rect.get_center()
	var route_line := PackedVector2Array()
	for point in game.city.points:
		route_line.append(center + (Vector2(point.x, point.z) - bounds.get_center()) * factor)
	draw_polyline(route_line, Color("6e9690"), 3, true)
	for index in range(game.city.stop_positions.size()):
		var point: Vector3 = game.city.stop_positions[index]
		var marker := center + (Vector2(point.x, point.z) - bounds.get_center()) * factor
		draw_circle(marker, 3.5, GOLD if index == game.next_stop else MUTED)
	var bus_point: Vector3 = game.bus.position
	var position := center + (Vector2(bus_point.x, bus_point.z) - bounds.get_center()) * factor
	var forward: Vector3 = -game.bus.global_basis.z
	var heading := Vector2(forward.x, forward.z)
	var side := heading.orthogonal()
	draw_colored_polygon(PackedVector2Array([position + heading * 8, position - heading * 5 + side * 5, position - heading * 5 - side * 5]), MINT)