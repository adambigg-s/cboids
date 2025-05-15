#define SOKOL_IMPL
#define SOKOL_D3D11

#include "../sokol/sokol_app.h"
#include "../sokol/sokol_gfx.h"
#include "../sokol/sokol_glue.h"
#include "../sokol/sokol_log.h"
#include "shaders.h"

#define WIDTH 800
#define HEIGHT 600

constexpr sg_color BACKGROUND_COLOR = sg_color{.r = 0.2, .g = 0.2, .b = 0.3};

static struct {
    sg_pass_action pass_action;
    sg_bindings bindings;
    sg_pipeline pipeline;
} state;

void init() {
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

    sg_buffer_desc buffer = {};
    buffer.size = sizeof(vertices);
    buffer.data = SG_RANGE(vertices);
    buffer.label = "triangle vertices";
    state.bindings.vertex_buffers[0] = sg_make_buffer(&buffer);

    sg_pipeline_desc pipeline = sg_pipeline_desc{};
    pipeline.shader = shader;
    pipeline.layout.attrs[ATTR_simple_position].format = SG_VERTEXFORMAT_FLOAT3;
    pipeline.label = "triangle pipeline";
    state.pipeline = sg_make_pipeline(pipeline);

    sg_pass_action pass_action = sg_pass_action{};
    pass_action.colors[0] =
        sg_color_attachment_action{.load_action = SG_LOADACTION_CLEAR, .clear_value = BACKGROUND_COLOR};
    state.pass_action = pass_action;
}

void frame() {
    sg_pass pass_action = sg_pass{};
    pass_action.action = state.pass_action;
    pass_action.swapchain = sglue_swapchain();
    sg_begin_pass(&pass_action);

    sg_apply_pipeline(state.pipeline);
    sg_apply_bindings(&state.bindings);
    sg_draw(0, 3, 1);

    sg_end_pass();
    sg_commit();
}

void event(const sapp_event *event) {
    if (event->type == SAPP_EVENTTYPE_KEY_DOWN && event->key_code == SAPP_KEYCODE_ESCAPE) {
        sapp_request_quit();
    }
}

void cleanup() {
    sg_shutdown();
}

sapp_desc sokol_main(int argc, char *argv[]) {
    sapp_desc description = sapp_desc{};
    description.init_cb = init;
    description.frame_cb = frame;
    description.event_cb = event;
    description.cleanup_cb = cleanup;
    description.width = WIDTH;
    description.height = HEIGHT;
    description.high_dpi = true;
    description.window_title = "boids simulation with Sokol";
    description.icon = sapp_icon_desc{.sokol_default = true};

    return description;
}
