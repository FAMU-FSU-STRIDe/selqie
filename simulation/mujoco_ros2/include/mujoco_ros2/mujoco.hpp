#pragma once

#include <functional>
#include <iostream>

// Headers for MuJoCo
#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

// Template for a MuJoCo control function
using MuJoCoControlFunction = std::function<void(const mjModel *model, mjData *data)>;

/*
 * Static data structure to hold MuJoCo data.
 * This structure contains the MuJoCo model, data, camera, scene, and other
 * rendering-related information.
 * Can be accessed from anywhere in the code.
 */
static struct
{
    GLFWwindow *window = nullptr; // GLFW window

    mjModel *model = nullptr; // MuJoCo model
    mjData *data = nullptr;   // MuJoCo data
    mjvCamera camera;         // Camera for rendering
    mjvPerturb perturb;       // Perturbation for rendering
    mjvOption option;         // Rendering options
    mjvScene scene;           // Scene for rendering
    mjrContext context;       // Rendering context

    /*
     * Structure to hold mouse state information
     */
    struct
    {
        bool button_left = false;
        bool button_middle = false;
        bool button_right = false;
        double lastx = 0;
        double lasty = 0;
    } mouse;

    // Structure to hold control functions
    // These functions are called during the control callback
    // to control the MuJoCo simulation
    // Add your control functions here
    std::vector<MuJoCoControlFunction> control_functions;
} MuJoCoData;

/*
 * Key press callback function
 * This function is called when a key is pressed in the GLFW window.
 */
static void keyPressCallback(GLFWwindow *, int key, int, int act, int)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        // Reset the MuJoCo simulation to its initial state if the backspace key is pressed
        mj_resetData(MuJoCoData.model, MuJoCoData.data);
        mj_forward(MuJoCoData.model, MuJoCoData.data);
    }
}

/*
 * Mouse click callback function
 * This function is called when a mouse button is clicked in the GLFW window.
 */
static void mouseClickCallback(GLFWwindow *window, int, int, int)
{
    // Update the mouse state and cursor position when a mouse button is clicked
    MuJoCoData.mouse.button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    MuJoCoData.mouse.button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    MuJoCoData.mouse.button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &MuJoCoData.mouse.lastx, &MuJoCoData.mouse.lasty);
}

/*
 * Mouse move callback function
 * This function is called when the mouse is moved in the GLFW window.
 */
static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos)
{
    // Check if any mouse button is pressed
    if (!MuJoCoData.mouse.button_left && !MuJoCoData.mouse.button_middle && !MuJoCoData.mouse.button_right)
        return;

    // Calculate the change in mouse position
    const double dx = xpos - MuJoCoData.mouse.lastx;
    const double dy = ypos - MuJoCoData.mouse.lasty;
    MuJoCoData.mouse.lastx = xpos;
    MuJoCoData.mouse.lasty = ypos;

    // Get the window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Check if the shift key is pressed
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                     (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // Determine the mouse action based on the mouse button pressed
    mjtMouse action;
    if (MuJoCoData.mouse.button_right)
    {
        // Right mouse button is used for camera movement
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (MuJoCoData.mouse.button_left)
    {
        // Left mouse button is used for camera rotation
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        // Middle mouse button is used for camera zoom
        action = mjMOUSE_ZOOM;
    }

    // Move the camera based on the mouse action
    mjv_moveCamera(MuJoCoData.model, action, dx / height, dy / height, &MuJoCoData.scene, &MuJoCoData.camera);
}

/*
 * Mouse scroll callback function
 * This function is called when the mouse wheel is scrolled in the GLFW window.
 */
static void mouseScrollCallback(GLFWwindow *, double, double yoffset)
{
    // Scroll the camera based on the mouse wheel movement
    mjv_moveCamera(MuJoCoData.model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &MuJoCoData.scene, &MuJoCoData.camera);
}

/*
 * Control callback function
 * This function is called during the MuJoCo simulation step.
 * It calls all registered control functions to control the simulation.
 */
static void controlCallback(const mjModel *model, mjData *data)
{
    for (const auto &control_function : MuJoCoData.control_functions)
    {
        // Call each control function with the model and data
        // This allows for custom control logic to be executed during the simulation
        control_function(model, data);
    }
}

/*
 * Initialize MuJoCo
 * This function loads the MuJoCo model from the specified XML file,
 * creates a GLFW window, and sets up the rendering context.
 */
static void initMuJoCo(const std::string model_path)
{
    // Load the MuJoCo model from the specified XML file
    char error[1000];
    MuJoCoData.model = mj_loadXML(model_path.c_str(), nullptr, error, 1000);

    // Check if the model was loaded successfully
    if (!MuJoCoData.model)
    {
        throw std::runtime_error("Error loading MuJoCo model: " + std::string(error));
    }

    // Create MuJoCo data
    MuJoCoData.data = mj_makeData(MuJoCoData.model);

    // Check if the data was created successfully
    if (!MuJoCoData.data)
    {
        mj_deleteModel(MuJoCoData.model);
        MuJoCoData.model = nullptr;
        throw std::runtime_error("Error making MuJoCo data");
    }

    // Initialize GLFW
    if (!glfwInit())
    {
        throw std::runtime_error("Error initializing GLFW");
    }

    // Create a GLFW window
    MuJoCoData.window = glfwCreateWindow(1200, 900, "MuJoCo", nullptr, nullptr);

    // Check if the window was created successfully
    if (!MuJoCoData.window)
    {
        glfwTerminate();
        throw std::runtime_error("Error creating GLFW window");
    }

    // Set the GLFW window to be the current context
    glfwMakeContextCurrent(MuJoCoData.window);
    glfwSwapInterval(1);

    // Set MuJoCo rendering options to default values
    mjv_defaultCamera(&MuJoCoData.camera);
    mjv_defaultPerturb(&MuJoCoData.perturb);
    mjv_defaultOption(&MuJoCoData.option);
    mjr_defaultContext(&MuJoCoData.context);

    // Make the MuJoCo scene and context
    mjv_makeScene(MuJoCoData.model, &MuJoCoData.scene, 1000);
    mjr_makeContext(MuJoCoData.model, &MuJoCoData.context, mjFONTSCALE_150);

    // Register the GLFW callbacks for key presses, mouse clicks, and mouse movements
    glfwSetKeyCallback(MuJoCoData.window, keyPressCallback);
    glfwSetCursorPosCallback(MuJoCoData.window, mouseMoveCallback);
    glfwSetMouseButtonCallback(MuJoCoData.window, mouseClickCallback);
    glfwSetScrollCallback(MuJoCoData.window, mouseScrollCallback);

    // Set the control callback function
    mjcb_control = controlCallback;
}

/*
 * Open MuJoCo
 * This function runs the MuJoCo simulation loop.
 * It updates the simulation state, renders the scene, and handles user input.
 */
static void openMuJoCo(const double frame_rate)
{
    // Run the MuJoCo simulation loop until the window is closed
    while (!glfwWindowShouldClose(MuJoCoData.window))
    {
        // Calculate the time step for the simulation
        const mjtNum simend = MuJoCoData.data->time + 1.0 / frame_rate;
        while (MuJoCoData.data->time < simend)
        {
            // Step the MuJoCo simulation until the next time step
            mj_step(MuJoCoData.model, MuJoCoData.data);
        }

        // Update the GLFW window size
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(MuJoCoData.window, &viewport.width, &viewport.height);

        // Update the MuJoCo scene and render it
        mjv_updateScene(MuJoCoData.model, MuJoCoData.data, &MuJoCoData.option,
                        &MuJoCoData.perturb, &MuJoCoData.camera, mjCAT_ALL, &MuJoCoData.scene);
        mjr_render(viewport, &MuJoCoData.scene, &MuJoCoData.context);

        // Swap the buffers to display the rendered scene
        // and poll for events (e.g., keyboard and mouse input)
        glfwSwapBuffers(MuJoCoData.window);
        glfwPollEvents();
    }
    // End of MuJoCo simulation loop

    // Clean up and close the MuJoCo simulation
    glfwDestroyWindow(MuJoCoData.window);

    // Free the MuJoCo scene and context
    mjv_freeScene(&MuJoCoData.scene);
    mjr_freeContext(&MuJoCoData.context);

    // Free the MuJoCo data and model
    mj_deleteData(MuJoCoData.data);
    mj_deleteModel(MuJoCoData.model);
}