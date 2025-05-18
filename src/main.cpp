#define SOKOL_IMPL
#define SOKOL_D3D11

const int WIDTH = 1920;
const int HEIGHT = 1080;

#include "../sokol/sokol_app.h"
#include "../sokol/sokol_gfx.h"
#include "../sokol/sokol_glue.h"
#include "../sokol/sokol_log.h"
#include "../sokol/sokol_time.h"

#include "boids.hpp"
#include "shaders.hpp"

constexpr sg_color BACKGROUND_COLOR = sg_color{.r = 0.2, .g = 0.2, .b = 0.3};

typedef struct State {
    sg_pass_action pass_action;
    sg_bindings bindings;
    sg_pipeline pipeline;
    World world;

    void update() {
        this->world.height = sapp_heightf();
        this->world.width = sapp_widthf();
        this->world.update();
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
    state->bindings.vertex_buffers[0] = sg_make_buffer(sg_buffer_desc{
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
    state->pipeline = sg_make_pipeline(pipeline_desc);

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
    sg_apply_pipeline(state->pipeline);
    sg_apply_bindings(&state->bindings);
    v_params_world_t world = v_params_world_t{
        .world_dims = {state->world.width, state->world.height},
    };
    sg_apply_uniforms(UB_v_params_world, sg_range{.ptr = &world, .size = sizeof(world)});
    for (Boid &boid : state->world.data.boids) {
        v_params_boid_t boid_params = v_params_boid_t{
            .pos = {boid.pos.x, boid.pos.y},
            .vel = {boid.vel.x, boid.vel.y},
            .scale = BOID_SCALE,
        };
        sg_apply_uniforms(UB_v_params_boid, sg_range{.ptr = &boid_params, .size = sizeof(boid_params)});
        sg_draw(0, 3, 1);
    }
    sg_end_pass();
    sg_commit();
}

void sok_event(const sapp_event *event, void *state_ptr) {
    if (event->type == SAPP_EVENTTYPE_KEY_DOWN) {
        if (event->key_code == SAPP_KEYCODE_ESCAPE) {
            sapp_request_quit();
        }
    }
}

void sok_cleanup(void *user_data) {
    State *state = (State *)user_data;

    sg_shutdown();
    delete state;
}

sapp_desc sokol_main(int _argc, char *_argv[]) {
    State *state_ptr = new State{};
    state_ptr->world = World{.width = WIDTH, .height = HEIGHT};

    for (int i = 0; i < BOID_COUNT; i += 1) {
        state_ptr->world.add_boid();
    }

    sapp_desc description = sapp_desc{
        .user_data = state_ptr,
        .init_userdata_cb = sok_init,
        .frame_userdata_cb = sok_frame,
        .cleanup_userdata_cb = sok_cleanup,
        .event_userdata_cb = sok_event,
        .width = WIDTH,
        .height = HEIGHT,
        .sample_count = 4,
        .high_dpi = true,
        .fullscreen = false,
        .window_title = "boids simulation with Sokol",
        .icon = sapp_icon_desc{.sokol_default = true},
        .logger = sapp_logger{.func = slog_func, .user_data = state_ptr},
        .win32_console_create = true,
    };

    return description;
}
