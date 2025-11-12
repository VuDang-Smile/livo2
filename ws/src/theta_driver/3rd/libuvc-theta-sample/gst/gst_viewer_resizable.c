/*
  Enhanced GStreamer Viewer với khả năng resize và scroll
  Dựa trên gst_viewer.c gốc với cải tiến cho camera 360
*/

#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <gtk/gtk.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "libuvc/libuvc.h"
#include "thetauvc.h"

#define MAX_PIPELINE_LEN 1024

struct gst_src {
    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *videoconvert;
    GstElement *videoscale;
    GstElement *videosink;
    GstElement *queue;
    
    GstElement *capsfilter;
    GstCaps *caps;
    
    GstBuffer *buffer;
    GstMapInfo map;
    
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    
    int width, height;
    int display_width, display_height;
    int enable_resize;
    int enable_scroll;
    int scroll_x, scroll_y;
    int zoom_level;
    
    pthread_t thread;
    int running;
};

static GtkWidget *window;
static GtkWidget *drawing_area;
static struct gst_src *gst_src_data;

// Callback cho mouse events
static gboolean on_mouse_press(GtkWidget *widget, GdkEventButton *event, gpointer user_data) {
    if (event->button == 1) { // Left click
        gst_src_data->scroll_x = event->x;
        gst_src_data->scroll_y = event->y;
        gst_src_data->enable_scroll = 1;
    }
    return TRUE;
}

static gboolean on_mouse_motion(GtkWidget *widget, GdkEventMotion *event, gpointer user_data) {
    if (gst_src_data->enable_scroll) {
        gst_src_data->scroll_x = event->x;
        gst_src_data->scroll_y = event->y;
    }
    return TRUE;
}

static gboolean on_mouse_release(GtkWidget *widget, GdkEventButton *event, gpointer user_data) {
    if (event->button == 1) {
        gst_src_data->enable_scroll = 0;
    }
    return TRUE;
}

static gboolean on_scroll(GtkWidget *widget, GdkEventScroll *event, gpointer user_data) {
    if (event->direction == GDK_SCROLL_UP) {
        gst_src_data->zoom_level = MIN(gst_src_data->zoom_level + 10, 200);
    } else if (event->direction == GDK_SCROLL_DOWN) {
        gst_src_data->zoom_level = MAX(gst_src_data->zoom_level - 10, 50);
    }
    printf("Zoom level: %d%%\n", gst_src_data->zoom_level);
    return TRUE;
}

static gboolean on_key_press(GtkWidget *widget, GdkEventKey *event, gpointer user_data) {
    switch (event->keyval) {
        case GDK_KEY_q:
        case GDK_KEY_Q:
            gtk_main_quit();
            break;
        case GDK_KEY_r:
        case GDK_KEY_R:
            gst_src_data->enable_resize = !gst_src_data->enable_resize;
            printf("Resize: %s\n", gst_src_data->enable_resize ? "ON" : "OFF");
            break;
        case GDK_KEY_1:
            gst_src_data->display_width = 1280;
            gst_src_data->display_height = 640;
            printf("Display size: %dx%d\n", gst_src_data->display_width, gst_src_data->display_height);
            break;
        case GDK_KEY_2:
            gst_src_data->display_width = 1920;
            gst_src_data->display_height = 960;
            printf("Display size: %dx%d\n", gst_src_data->display_width, gst_src_data->display_height);
            break;
        case GDK_KEY_3:
            gst_src_data->display_width = 2560;
            gst_src_data->display_height = 1280;
            printf("Display size: %dx%d\n", gst_src_data->display_width, gst_src_data->display_height);
            break;
        case GDK_KEY_plus:
        case GDK_KEY_equal:
            gst_src_data->display_width = MIN(gst_src_data->display_width + 160, 2560);
            gst_src_data->display_height = MIN(gst_src_data->display_height + 80, 1280);
            printf("Display size: %dx%d\n", gst_src_data->display_width, gst_src_data->display_height);
            break;
        case GDK_KEY_minus:
            gst_src_data->display_width = MAX(gst_src_data->display_width - 160, 640);
            gst_src_data->display_height = MAX(gst_src_data->display_height - 80, 320);
            printf("Display size: %dx%d\n", gst_src_data->display_width, gst_src_data->display_height);
            break;
    }
    return TRUE;
}

// Callback cho draw event
static gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer user_data) {
    // Vẽ frame từ GStreamer pipeline
    // Implementation sẽ được thêm sau
    return TRUE;
}

static void create_ui() {
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Enhanced Theta X Viewer - Resizable & Scrollable");
    gtk_window_set_default_size(GTK_WINDOW(window), 1280, 640);
    gtk_window_set_resizable(GTK_WINDOW(window), TRUE);
    
    drawing_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(drawing_area, 1280, 640);
    
    // Kết nối events
    g_signal_connect(drawing_area, "draw", G_CALLBACK(on_draw), NULL);
    g_signal_connect(drawing_area, "button-press-event", G_CALLBACK(on_mouse_press), NULL);
    g_signal_connect(drawing_area, "motion-notify-event", G_CALLBACK(on_mouse_motion), NULL);
    g_signal_connect(drawing_area, "button-release-event", G_CALLBACK(on_mouse_release), NULL);
    g_signal_connect(drawing_area, "scroll-event", G_CALLBACK(on_scroll), NULL);
    g_signal_connect(window, "key-press-event", G_CALLBACK(on_key_press), NULL);
    
    // Enable events
    gtk_widget_set_events(drawing_area, 
        GDK_BUTTON_PRESS_MASK | 
        GDK_BUTTON_RELEASE_MASK | 
        GDK_POINTER_MOTION_MASK | 
        GDK_SCROLL_MASK);
    
    gtk_widget_set_can_focus(drawing_area, TRUE);
    gtk_widget_grab_focus(drawing_area);
    
    gtk_container_add(GTK_CONTAINER(window), drawing_area);
    gtk_widget_show_all(window);
}

static void *gst_thread_func(void *arg) {
    struct gst_src *src = (struct gst_src *)arg;
    
    while (src->running) {
        // Đọc frame từ camera và đưa vào GStreamer pipeline
        // Implementation sẽ được thêm sau
        usleep(33000); // ~30 FPS
    }
    
    return NULL;
}

int main(int argc, char *argv[]) {
    // Initialize GTK
    gtk_init(&argc, &argv);
    
    // Initialize GStreamer
    gst_init(&argc, &argv);
    
    // Allocate memory
    gst_src_data = g_malloc0(sizeof(struct gst_src));
    gst_src_data->display_width = 1280;
    gst_src_data->display_height = 640;
    gst_src_data->enable_resize = 1;
    gst_src_data->enable_scroll = 0;
    gst_src_data->scroll_x = 0;
    gst_src_data->scroll_y = 0;
    gst_src_data->zoom_level = 100;
    gst_src_data->running = 1;
    
    // Create UI
    create_ui();
    
    // Start GStreamer thread
    pthread_create(&gst_src_data->thread, NULL, gst_thread_func, gst_src_data);
    
    // Show help
    printf("\n=== Enhanced Theta X Viewer ===\n");
    printf("Phím tắt:\n");
    printf("  q = Thoát\n");
    printf("  r = Toggle resize\n");
    printf("  1 = 1280x640, 2 = 1920x960, 3 = 2560x1280\n");
    printf("  +/- = Tăng/giảm kích thước\n");
    printf("  Mouse drag = Scroll\n");
    printf("  Mouse wheel = Zoom\n");
    printf("===============================\n\n");
    
    // Run GTK main loop
    gtk_main();
    
    // Cleanup
    gst_src_data->running = 0;
    pthread_join(gst_src_data->thread, NULL);
    g_free(gst_src_data);
    
    return 0;
}

