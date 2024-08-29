#pragma once

#include <functional>
#include <iostream>
#include <mutex>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

using MuJoCoControlFunction = std::function<void(const mjModel *model, mjData *data)>;

static struct
{
    GLFWwindow *window = nullptr;
    mjModel *model = nullptr;
    mjData *data = nullptr;
    mjvCamera camera;
    mjvPerturb perturb;
    mjvOption option;
    mjvScene scene;
    mjrContext context;

    struct
    {
        bool button_left = false;
        bool button_middle = false;
        bool button_right = false;
        double lastx = 0;
        double lasty = 0;
    } mouse;

    std::vector<MuJoCoControlFunction> control_functions;

    std::mutex mutex;
    
} MuJoCoData;

static void keyPressCallback(GLFWwindow *, int key, int, int act, int)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(MuJoCoData.model, MuJoCoData.data);
        mj_forward(MuJoCoData.model, MuJoCoData.data);
    }
}

static void mouseClickCallback(GLFWwindow *window, int, int, int)
{
    MuJoCoData.mouse.button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    MuJoCoData.mouse.button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    MuJoCoData.mouse.button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

    glfwGetCursorPos(window, &MuJoCoData.mouse.lastx, &MuJoCoData.mouse.lasty);
}

static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos)
{
    if (!MuJoCoData.mouse.button_left && !MuJoCoData.mouse.button_middle && !MuJoCoData.mouse.button_right)
        return;

    const double dx = xpos - MuJoCoData.mouse.lastx;
    const double dy = ypos - MuJoCoData.mouse.lasty;
    MuJoCoData.mouse.lastx = xpos;
    MuJoCoData.mouse.lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                     (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (MuJoCoData.mouse.button_right)
    {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (MuJoCoData.mouse.button_left)
    {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(MuJoCoData.model, action, dx / height, dy / height, &MuJoCoData.scene, &MuJoCoData.camera);
}

static void mouseScrollCallback(GLFWwindow *, double, double yoffset)
{
    mjv_moveCamera(MuJoCoData.model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &MuJoCoData.scene, &MuJoCoData.camera);
}

static void controlCallback(const mjModel *model, mjData *data)
{
    for (const auto &control_function : MuJoCoData.control_functions)
    {
        control_function(model, data);
    }
}

static void initMuJoCo(const std::string model_path)
{
    char error[1000];
    MuJoCoData.model = mj_loadXML(model_path.c_str(), nullptr, error, 1000);

    if (!MuJoCoData.model)
    {
        throw std::runtime_error("Error loading MuJoCo model: " + std::string(error));
    }

    MuJoCoData.data = mj_makeData(MuJoCoData.model);

    if (!MuJoCoData.data)
    {
        mj_deleteModel(MuJoCoData.model);
        MuJoCoData.model = nullptr;
        throw std::runtime_error("Error making MuJoCo data");
    }

    if (!glfwInit())
    {
        throw std::runtime_error("Error initializing GLFW");
    }

    MuJoCoData.window = glfwCreateWindow(1200, 900, "MuJoCo", nullptr, nullptr);

    if (!MuJoCoData.window)
    {
        glfwTerminate();
        throw std::runtime_error("Error creating GLFW window");
    }

    glfwMakeContextCurrent(MuJoCoData.window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&MuJoCoData.camera);
    mjv_defaultPerturb(&MuJoCoData.perturb);
    mjv_defaultOption(&MuJoCoData.option);
    mjr_defaultContext(&MuJoCoData.context);

    mjv_makeScene(MuJoCoData.model, &MuJoCoData.scene, 1000);
    mjr_makeContext(MuJoCoData.model, &MuJoCoData.context, mjFONTSCALE_150);

    glfwSetKeyCallback(MuJoCoData.window, keyPressCallback);
    glfwSetCursorPosCallback(MuJoCoData.window, mouseMoveCallback);
    glfwSetMouseButtonCallback(MuJoCoData.window, mouseClickCallback);
    glfwSetScrollCallback(MuJoCoData.window, mouseScrollCallback);

    mjcb_control = controlCallback;
}

static void openMuJoCo(const double frame_rate)
{
    while (!glfwWindowShouldClose(MuJoCoData.window))
    {
        const mjtNum simend = MuJoCoData.data->time + 1.0 / frame_rate;
        while (MuJoCoData.data->time < simend)
        {
            mj_step(MuJoCoData.model, MuJoCoData.data);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(MuJoCoData.window, &viewport.width, &viewport.height);

        mjv_updateScene(MuJoCoData.model, MuJoCoData.data, &MuJoCoData.option,
                        &MuJoCoData.perturb, &MuJoCoData.camera, mjCAT_ALL, &MuJoCoData.scene);
        mjr_render(viewport, &MuJoCoData.scene, &MuJoCoData.context);

        glfwSwapBuffers(MuJoCoData.window);
        glfwPollEvents();
    }

    glfwDestroyWindow(MuJoCoData.window);

    mjv_freeScene(&MuJoCoData.scene);
    mjr_freeContext(&MuJoCoData.context);

    mj_deleteData(MuJoCoData.data);
    mj_deleteModel(MuJoCoData.model);
}