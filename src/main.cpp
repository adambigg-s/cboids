#define SOKOL_IMPL
#define SOKOL_D3D11

#include "../sokol/sokol_app.h"
#include "../sokol/sokol_gfx.h"
#include "../sokol/sokol_glue.h"
#include "../sokol/sokol_log.h"
#include "shaders.hpp"

#define WIDTH 800
#define HEIGHT 600

constexpr sg_color BACKGROUND_COLOR = sg_color{.r = 0.2, .g = 0.2, .b = 0.3};

typedef struct State {
    sg_pass_action pass_action;
    sg_bindings bindings;
    sg_pipeline pipeline;
} State;

void init(void *state_ptr) {
    State *state = (State *)state_ptr;

    sg_desc description = sg_desc{};
    description.environment = sglue_environment();
    description.logger = sg_logger{.func = slog_func};
    sg_setup(&description);

    sg_shader shader = sg_make_shader(simple_shader_desc(sg_query_backend()));

    // clang-format off
    float vertices[] = {
        -0.5, -0.5, 0.0,
        0.5, -0.5, 0.0,
        0.0, 0.5, 0.0,
    };
    // clang-format on
    sg_buffer_desc buffer_desc = {};
    buffer_desc.size = sizeof(vertices);
    buffer_desc.data = sg_range{.ptr = &vertices, .size = sizeof(vertices)};
    buffer_desc.label = "triangle vertices";
    state->bindings.vertex_buffers[0] = sg_make_buffer(&buffer_desc);

    sg_pipeline_desc pipe_desc = sg_pipeline_desc{};
    pipe_desc.shader = shader;
    pipe_desc.layout.attrs[ATTR_simple_position].format = SG_VERTEXFORMAT_FLOAT3;
    pipe_desc.label = "triangle pipeline";
    state->pipeline = sg_make_pipeline(pipe_desc);

    sg_pass_action pass_action = sg_pass_action{};
    pass_action.colors[0] =
        sg_color_attachment_action{.load_action = SG_LOADACTION_CLEAR, .clear_value = BACKGROUND_COLOR};
    state->pass_action = pass_action;
}

void frame(void *state_ptr) {
    State *state = (State *)state_ptr;

    sg_pass pass = sg_pass{};
    pass.action = state->pass_action;
    pass.swapchain = sglue_swapchain();
    sg_begin_pass(&pass);

    sg_apply_pipeline(state->pipeline);
    sg_apply_bindings(&state->bindings);
    sg_draw(0, 3, 1);

    sg_end_pass();
    sg_commit();
}

void event(const sapp_event *event, void *state_ptr) {
    State *state = (State *)state_ptr;

    if (event->type == SAPP_EVENTTYPE_KEY_DOWN && event->key_code == SAPP_KEYCODE_ESCAPE) {
        sapp_request_quit();
    }
}

void cleanup(void *user_data) {
    State *state = (State *)user_data;

    sg_shutdown();
    free(user_data);
}

sapp_desc sokol_main(int _argc, char *_argv[]) {
    State *state_ptr = (State *)malloc(sizeof(State));
    *state_ptr = State{};

    sapp_desc description = sapp_desc{};
    description.user_data = state_ptr;
    description.init_userdata_cb = init;
    description.frame_userdata_cb = frame;
    description.event_userdata_cb = event;
    description.cleanup_userdata_cb = cleanup;
    description.width = WIDTH;
    description.height = HEIGHT;
    description.high_dpi = true;
    description.window_title = "boids simulation with Sokol";
    description.icon = sapp_icon_desc{.sokol_default = true};
    description.fullscreen = false;
    description.sample_count = 4;
    description.logger = sapp_logger{.func = slog_func};

    return description;
}
