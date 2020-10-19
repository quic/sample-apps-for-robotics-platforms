/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>

#include <wayland-client.h>
#include <wayland-egl.h>
#include <wayland-cursor.h>

// This Sample-code describes how to create a simple window in Weston
// Run the program and enter the following parameters
// -r : Show the red window
// -g : Show the green window
// -b : Show the blue window

struct WaylandGlobals {
    struct wl_compositor* compositor;
    struct wl_shell* shell;
    struct wl_seat *seat;
    struct wl_keyboard *keyboard;
    struct wl_pointer *pointer;
};

EGLDisplay eglDisplay;
EGLSurface eglSurface;

// Bind the basic interface
void registry_global(void* data, struct wl_registry* registry,
                            uint32_t id, const char* interface, uint32_t version)
{
    printf("interface = %s\n",interface);
    struct WaylandGlobals* globals = (struct WaylandGlobals *)data;
    if (strcmp(interface, "wl_compositor") == 0) {
        globals->compositor = (struct wl_compositor *)wl_registry_bind(registry, id, &wl_compositor_interface, 1);
    } else if (strcmp(interface, "wl_shell") == 0) {
        globals->shell = (struct wl_shell *)wl_registry_bind(registry, id, &wl_shell_interface, 1);
    }
}
const struct wl_registry_listener registry_listener = { registry_global, NULL };

// Wayland Init
void initWaylandDisplay(struct wl_display** wlDisplay, struct wl_surface** wlSurface)
{
    struct WaylandGlobals globals = {0};
    // Link wayland service
    *wlDisplay = wl_display_connect(NULL);
    assert(*wlDisplay != NULL);
    //Get all the basic interfaces provided by the server
    struct wl_registry* registry = wl_display_get_registry(*wlDisplay);
    wl_registry_add_listener(registry, &registry_listener, (void *) &globals);
    printf("wl_registry_add_listener : OK\n");

    wl_display_dispatch(*wlDisplay);
    wl_display_roundtrip(*wlDisplay);
    assert(globals.compositor);
    assert(globals.shell);
    // Create surface
    *wlSurface = wl_compositor_create_surface(globals.compositor);
    assert(*wlSurface != NULL);

    struct wl_shell_surface* shellSurface = wl_shell_get_shell_surface(globals.shell, *wlSurface);
    wl_shell_surface_set_toplevel(shellSurface);
}
// EGL Init
void initEGLDisplay(EGLNativeDisplayType nativeDisplay, EGLNativeWindowType nativeWindow,
                    EGLDisplay* eglDisp, EGLSurface* eglSurf)
{
    EGLint number_of_config;
    // Choose the optimal configuration based on the specified attributes
    EGLint config_attribs[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_NONE
    };

    const EGLint context_attribs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 2,
        EGL_NONE
    };

    // Get the foreground window and initialize Display
    *eglDisp = eglGetDisplay(nativeDisplay);
    assert(*eglDisp != EGL_NO_DISPLAY);
    EGLBoolean initialized = eglInitialize(*eglDisp, NULL, NULL);
    assert(initialized == EGL_TRUE);

    EGLConfig configs[1];

    eglChooseConfig(*eglDisp, config_attribs, configs, 1, &number_of_config);
    assert(number_of_config);
    // Create EGL context with configuration
    EGLContext eglContext = eglCreateContext(*eglDisp, configs[0], EGL_NO_CONTEXT, context_attribs);
    // Create Surface with selected translation configuration
    *eglSurf = eglCreateWindowSurface(*eglDisp, configs[0], nativeWindow, NULL);
    assert(*eglSurf != EGL_NO_SURFACE);

    EGLBoolean makeCurrent = eglMakeCurrent(*eglDisp, *eglSurf, *eglSurf, eglContext);
    assert(makeCurrent == EGL_TRUE);
}

void initWindow(GLint width, GLint height, struct wl_display** wlDisplay)
{
    struct wl_surface* wlSurface;
    initWaylandDisplay(wlDisplay, &wlSurface);

    struct wl_egl_window* wlEglWindow = wl_egl_window_create(wlSurface, width, height);
    assert(wlEglWindow != NULL);

    initEGLDisplay((EGLNativeDisplayType) *wlDisplay, (EGLNativeWindowType) wlEglWindow, &eglDisplay, &eglSurface);
}

int
main(int argc, char **argv)
{
    int i = 0;
    int num = 0;
    char input[16] = {0};
    float clr[3] = {0};
    int ret = 0;

    int width = 480;
    int height = 540;
    struct wl_display* wlDisplay;
    // window Init
    initWindow(width, height, &wlDisplay);
    // Choose display color according to parameters
    int msg_size = 1;
    if (argc == 1){
        printf("command option missing, please refer to the Readme document!\n");
    }
    else if(strcmp(argv[1],"-r") == 0){
        clr[0] = 1.0f;
    }
    else if(strcmp(argv[1],"-g") == 0){
        clr[1] = 1.0f;
    }
    else if(strcmp(argv[1],"-b") == 0){
        clr[2] = 1.0f;
    }else{
        printf("Option not recognized, please refer to the Readme document!\n");
    }

    while (strcmp(input,"stop") != 0) {
        wl_display_dispatch_pending(wlDisplay);
        glClearColor(clr[0], clr[1], clr[2], 1.0f);
        glClear( GL_COLOR_BUFFER_BIT );
        eglSwapBuffers(eglDisplay, eglSurface);

        if (num >= 5) {
            memset(input, 0x00, 16);
            printf("if you want to close window,input 'stop'.\n please input string:");
            scanf("%s", input);
        }
        num++;
    }

    printf("simple-egl exiting\n");
    wl_display_disconnect(wlDisplay);

    return 0;
}
