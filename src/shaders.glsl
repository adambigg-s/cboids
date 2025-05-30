///////////////////////////////////////////////////////////////////////////////////////////////////
// simple shader 2d
@vs simple_vs
#define PI 3.141592
        
in vec2 v_pos;
in vec3 v_color;

out vec3 f_color;
out float f_angle;

layout (binding = 0) uniform v_params_boid {
    vec2 pos;
    vec2 vel;
    float scale;
};

layout (binding = 1) uniform v_params_world {
    vec2 world_dims;
};

void main() {
    f_color = v_color;
    f_angle = atan(vel.y, vel.x);
    
    vec2 local = v_pos * scale;
    
    float angle = f_angle - PI / 2;
    float rotx = local.x * cos(angle) - local.y * sin(angle);
    float roty = local.x * sin(angle) + local.y * cos(angle);
    vec2 rotated = vec2(rotx, roty);

    vec2 world = pos + rotated;

    float ndcx = (world.x / world_dims.x) * 2. - 1.;
    float ndcy = (world.y / world_dims.y) * 2. - 1.;
    vec2 ndc = vec2(ndcx, ndcy);

    gl_Position = vec4(ndc, 0., 1.);
}
@end

@fs simple_fs
#define PI 3.141592
#define TAU PI * 2
        
in vec3 f_color;
in float f_angle;

out vec3 color;

void main() {
    vec3 color_alter = vec3(
        abs(sin(f_angle)),
        abs(sin(f_angle + TAU / 3.)),
        abs(sin(f_angle + TAU / 3. * 2.))
    );

    float magic = sqrt(2) / 2;
    color = mix(f_color, color_alter, 1 - magic);
}
@end

@program simple simple_vs simple_fs
///////////////////////////////////////////////////////////////////////////////////////////////////
