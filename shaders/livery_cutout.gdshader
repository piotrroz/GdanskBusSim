shader_type spatial;
render_mode cull_back;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic;

void fragment() {
	vec4 base = texture(albedo_texture, UV);
	// Cut-outs are exact black in the template; mipmaps blur them, so test the sharpest level too.
	vec3 sharp = textureLod(albedo_texture, UV, 0.0).rgb;
	if (max(max(sharp.r, sharp.g), sharp.b) < 0.012) {
		discard;
	}
	ALBEDO = base.rgb;
	ROUGHNESS = 0.38;
}
