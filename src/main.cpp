#define SOKOL_IMPL
#define SOKOL_D3D11

#include "../sokol/sokol_app.h"
#include "../sokol/sokol_gfx.h"
#include "../sokol/sokol_glue.h"
#include "../sokol/sokol_log.h"

#include "boids.hpp"
#include "shaders.hpp"

constexpr sg_color BACKGROUND_COLOR = sg_color{.r = 0.15, .g = 0.15, .b = 0.25};

typedef struct State {
    sg_pass_action pass_action;
    sg_bindings boid_binding;
    sg_pipeline boid_pipeline;
    float frame_time;

    World world;

    void update() {
        this->world.bounds.ymax = sapp_heightf();
        this->world.bounds.xmax = sapp_widthf();
        this->world.update(this->frame_time);
    }
} State;

void sok_init(void *state_ptr) {
    State *state = (State *)state_ptr;

    sg_setup(sg_desc{
        .logger = sg_logger{.func = slog_func, .user_data = state_ptr},
        .environment = sglue_environment(),
    });

    // clang-format off
    float vertices[] = {
        -0.4, -0.4,    0.7, 1.0, 0.0,
         0.4, -0.4,    0.0, 0.7, 1.0,
         0.0,  1.0,    1.0, 0.0, 0.7,
    };
    // clang-format on
    state->boid_binding.vertex_buffers[0] = sg_make_buffer(sg_buffer_desc{
        .size = sizeof(vertices),
        .type = sg_buffer_type::SG_BUFFERTYPE_VERTEXBUFFER,
        .data = sg_range{.ptr = &vertices, .size = sizeof(vertices)},
        .label = "boid vertices",
    });

    sg_pipeline_desc pipeline_desc = sg_pipeline_desc{
        .shader = sg_make_shader(simple_shader_desc(sg_query_backend())),
        .label = "boid pipeline",
    };
    pipeline_desc.layout.attrs[ATTR_simple_v_pos].format = SG_VERTEXFORMAT_FLOAT2;
    pipeline_desc.layout.attrs[ATTR_simple_v_color].format = SG_VERTEXFORMAT_FLOAT3;
    state->boid_pipeline = sg_make_pipeline(pipeline_desc);

    state->pass_action = sg_pass_action{};
    state->pass_action.colors[0] = sg_color_attachment_action{
        .load_action = SG_LOADACTION_CLEAR,
        .clear_value = BACKGROUND_COLOR,
    };
}

void sok_frame(void *state_ptr) {
    State *state = (State *)state_ptr;

    state->update();

    sg_begin_pass(sg_pass{
        .action = state->pass_action,
        .swapchain = sglue_swapchain(),
    });
    sg_apply_pipeline(state->boid_pipeline);
    sg_apply_bindings(&state->boid_binding);
    v_params_world_t world = v_params_world_t{
        .world_dims = {state->world.bounds.xmax, state->world.bounds.ymax},
    };
    sg_apply_uniforms(UB_v_params_world, sg_range{.ptr = &world, .size = sizeof(world)});
    for (Boid &boid : state->world.data.boids) {
        v_params_boid_t boid_params = v_params_boid_t{
            .pos = {boid.position.x, boid.position.y},
            .vel = {boid.velocity.x, boid.velocity.y},
            .scale = state->world.data.params.boid_scale,
        };
        sg_apply_uniforms(UB_v_params_boid, sg_range{.ptr = &boid_params, .size = sizeof(boid_params)});
        sg_draw(0, state->world.data.params.vertices, 1);
    }
    sg_end_pass();
    sg_commit();
}

void sok_event(const sapp_event *event, void *state_ptr) {
    State *state = (State *)state_ptr;

    if (event->type == SAPP_EVENTTYPE_KEY_DOWN) {
        if (event->key_code == SAPP_KEYCODE_ESCAPE) {
            sapp_request_quit();
        }

        if (event->key_code == SAPP_KEYCODE_Q) {
            state->world.data.params.boid_count *= 2;
        } else if (event->key_code == SAPP_KEYCODE_A) {
            state->world.data.params.boid_count /= 2;
        }
        printf("boid count: %d\n", state->world.data.params.boid_count);

        if (event->key_code == SAPP_KEYCODE_W) {
            state->world.data.params.neighbor_distance *= 2;
        } else if (event->key_code == SAPP_KEYCODE_S) {
            state->world.data.params.neighbor_distance /= 2;
        }
        printf("neighbor distance: %.2f\n", state->world.data.params.neighbor_distance);

        if (event->key_code == SAPP_KEYCODE_E) {
            state->world.data.params.separation_distance *= 2;
        } else if (event->key_code == SAPP_KEYCODE_D) {
            state->world.data.params.separation_distance /= 2;
        }
        printf("separation distance: %.2f\n", state->world.data.params.separation_distance);

        if (event->key_code == SAPP_KEYCODE_R) {
            state->world.data.params.alignment *= 2;
        } else if (event->key_code == SAPP_KEYCODE_F) {
            state->world.data.params.alignment /= 2;
        }
        printf("alignment constant: %.2f\n", state->world.data.params.alignment);

        if (event->key_code == SAPP_KEYCODE_T) {
            state->world.data.params.cohesion *= 2;
        } else if (event->key_code == SAPP_KEYCODE_G) {
            state->world.data.params.cohesion /= 2;
        }
        printf("cohesion constant: %.2f\n", state->world.data.params.cohesion);

        if (event->key_code == SAPP_KEYCODE_Y) {
            state->world.data.params.separation *= 2;
        } else if (event->key_code == SAPP_KEYCODE_H) {
            state->world.data.params.separation /= 2;
        }
        printf("separation constant: %.2f\n", state->world.data.params.separation);
    }
}

void sok_cleanup(void *user_data) {
    State *state = (State *)user_data;

    sg_shutdown();
    delete state;
}

sapp_desc sokol_main(int _argc, char *_argv[]) {
    State *state_ptr = new State{};
    state_ptr->frame_time = 0.05;
    state_ptr->world = World{.bounds = BoundingBox{.xmin = 0, .xmax = 1920, .ymin = 0, .ymax = 1080}};
    state_ptr->world.data.params = BoidParams{
        .vertices = 3,
        .boid_count = 500,
        .max_speed = 200,
        .min_speed = 75,
        .boid_scale = 10,
        .neighbor_distance = 200,
        .separation_distance = 50,
        .cohesion = 0.625,
        .alignment = 2.5,
        .separation = 1000,
        .peripheral_angle = PI / 6,
        .wall_distance = 300,
        .wall_strength = 100000,
    };

    sapp_desc description = sapp_desc{
        .user_data = state_ptr,
        .init_userdata_cb = sok_init,
        .frame_userdata_cb = sok_frame,
        .cleanup_userdata_cb = sok_cleanup,
        .event_userdata_cb = sok_event,
        .width = (int)state_ptr->world.bounds.xmax,
        .height = (int)state_ptr->world.bounds.ymax,
        .sample_count = 4,
        .high_dpi = true,
        .fullscreen = false,
        .window_title = "boids flocking simulation",
        .icon = sapp_icon_desc{.sokol_default = true},
        .logger = sapp_logger{.func = slog_func, .user_data = state_ptr},
        .win32_console_create = true,
    };

    return description;
}
