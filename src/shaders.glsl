///////////////////////////////////////////////////////////////////////////////////////////////////
// simple shader 2d
@vs simple_vs
in vec2 v_pos;
in vec3 v_color;

out vec3 f_color;
out float f_rotation;

layout (binding = 0) uniform v_params_boid {
    float xpos;
    float ypos;
    float angle;
    float scale;
};

layout (binding = 1) uniform v_params_world {
    float worldx;
    float worldy;
};

void main() {
    f_color = v_color;
    f_rotation = angle;
    
    vec2 local = v_pos * scale;
    
    float rotx = local.x * cos(angle) - local.y * sin(angle);
    float roty = local.x * sin(angle) + local.y * cos(angle);
    vec2 rotated = vec2(rotx, roty);

    vec2 world = vec2(xpos, ypos) + rotated;

    float ndcx = (world.x / worldx) * 2. - 1.;
    float ndcy = (world.y / worldy) * 2. - 1.;

    vec2 ndc = vec2(ndcx, ndcy);

    gl_Position = vec4(ndc, 0., 1.);
}
@end

@fs simple_fs
#define TAU 3.141592 * 2
        
in vec3 f_color;
in float f_rotation;

out vec3 color;

void main() {
    vec3 color_alter = vec3(
        abs(sin(f_rotation)),
        abs(sin(f_rotation + TAU / 3)),
        abs(sin(f_rotation + 2 * TAU / 3))
    );

    color = mix(f_color, color_alter, 0.33);
}
@end

@program simple simple_vs simple_fs
///////////////////////////////////////////////////////////////////////////////////////////////////
