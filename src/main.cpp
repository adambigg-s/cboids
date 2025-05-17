#define SOKOL_IMPL
#define SOKOL_D3D11

#define PI 3.14159265

#include <vector>

#include "../sokol/sokol_app.h"
#include "../sokol/sokol_gfx.h"
#include "../sokol/sokol_glue.h"
#include "../sokol/sokol_log.h"

#include "shaders.hpp"

constexpr sg_color BACKGROUND_COLOR = sg_color{.r = 0.2, .g = 0.2, .b = 0.3};
constexpr int WIDTH = 1920;
constexpr int HEIGHT = 1080;
constexpr int BOID_COUNT = 3000;
constexpr int BOID_SCALE = 15;

typedef struct Boid {
    float direc;
    float x;
    float y;
    float speed;

    static Boid build(float x, float y) {
        return Boid{
            .direc = PI / 2,
            .x = x,
            .y = y,
            .speed = 10,
        };
    }

    void move() {
        this->x += this->speed * -sin(this->direc);
        this->y += this->speed * cos(this->direc);

        if (rand() % 10 > 1) {
            this->direc += 0.01;
        }
        if (rand() % 10 > 1) {
            this->direc -= 0.01;
        }
    }

    void contain(float xmin, float xmax, float ymin, float ymax) {
        float xrange = xmax - xmin;
        float yrange = ymax - ymin;
        if (this->x > xmax) {
            this->x -= xrange;
        } else if (this->x < xmin) {
            this->x += xrange;
        }
        if (this->y > ymax) {
            this->y -= yrange;
        } else if (this->y < ymin) {
            this->y += yrange;
        }
    }
} Boid;

typedef struct World {
    float width;
    float height;
    std::vector<Boid> boids;

    void add_boid() {
        int x = rand() % (int)this->width;
        int y = rand() % (int)this->height;
        this->boids.push_back(Boid::build(x, y));
    }

    void update() {
        for (Boid &boid : this->boids) {
            boid.move();
            boid.contain(0., this->width, 0., this->height);
        }
    }
} World;

typedef struct State {
    sg_pass_action pass_action;
    sg_bindings bindings;
    sg_pipeline pipeline;
    World world;

    void update() {
        this->world.height = sapp_heightf();
        this->world.width = sapp_widthf();
    }
} State;

void init(void *state_ptr) {
    State *state = (State *)state_ptr;

    sg_setup(sg_desc{
        .logger = sg_logger{.func = slog_func, .user_data = state_ptr},
        .environment = sglue_environment(),
    });

    // clang-format off
    float vertices[] = {
        -0.4, -0.4,    0.7, 1.,  0.,
        0.4,  -0.4,    0.,  0.7, 1.,
        0.,   1.,      1.,  0.,  0.7,
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

void frame(void *state_ptr) {
    State *state = (State *)state_ptr;

    state->update();
    state->world.update();

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
    for (Boid &boid : state->world.boids) {
        v_params_boid_t boid_params = v_params_boid_t{
            .pos = {boid.x, boid.y},
            .angle = boid.direc,
            .scale = BOID_SCALE,
        };
        sg_apply_uniforms(UB_v_params_boid, sg_range{.ptr = &boid_params, .size = sizeof(boid_params)});
        sg_draw(0, 3, 1);
    }
    sg_end_pass();
    sg_commit();
}

void event(const sapp_event *event, void *state_ptr) {
    if (event->type == SAPP_EVENTTYPE_KEY_DOWN) {
        if (event->key_code == SAPP_KEYCODE_ESCAPE) {
            sapp_request_quit();
        }
    }
}

void cleanup(void *user_data) {
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
    description.logger = sapp_logger{.func = slog_func, .user_data = state_ptr};
    description.win32_console_create = true;

    return description;
}
