/*
  Copyright 2020 K. Takeo. All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  3. Neither the name of the author nor other contributors may be
  used to endorse or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

 */

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>


#include "libuvc/libuvc.h"
#include "thetauvc.h"
#include "theta_log.h"

#define MAX_PIPELINE_LEN 1024

struct gst_src {
	GstElement *pipeline;
	GstElement *appsrc;

	GMainLoop *loop;
	GTimer *timer;
	guint framecount;
	guint id;
	guint bus_watch_id;
	uint32_t dwFrameInterval;
	uint32_t dwClockFrequency;
};

struct gst_src src;

static gboolean
gst_bus_cb(GstBus *bus, GstMessage *message, gpointer data)
{
	GError *err;
	gchar *dbg;

	switch (GST_MESSAGE_TYPE(message)) {
	case GST_MESSAGE_ERROR:
		gst_message_parse_error(message, &err, &dbg);
		THETA_LOG_ERROR("gst_bus_cb: GStreamer pipeline error: %s", err->message);
		if (dbg && *dbg)
			THETA_LOG_ERROR("gst_bus_cb: debug info: %s", dbg);
		g_error_free(err);
		g_free(dbg);
		g_main_loop_quit(src.loop);
		break;
	case GST_MESSAGE_WARNING:
		gst_message_parse_warning(message, &err, &dbg);
		THETA_LOG_WARNING("gst_bus_cb: GStreamer pipeline warning: %s", err->message);
		if (dbg && *dbg)
			THETA_LOG_WARNING("gst_bus_cb: debug info: %s", dbg);
		g_error_free(err);
		g_free(dbg);
		break;
	case GST_MESSAGE_EOS:
		THETA_LOG_INFO("gst_bus_cb: GStreamer pipeline EOS (end of stream)");
		g_main_loop_quit(src.loop);
		break;
	default:
		/* Skip logging frequent messages (e.g. STATE_CHANGED=64) to avoid log spam */
		break;
	}

	return TRUE;
}


int
gst_src_init(int *argc, char ***argv, char *pipeline)
{
	GstCaps *caps;
	GstBus *bus;
	char pipeline_str[MAX_PIPELINE_LEN];
	GError *parse_err = NULL;

	THETA_LOG_INFO("gst_src_init: initializing pipeline");

	snprintf(pipeline_str, MAX_PIPELINE_LEN, "appsrc name=ap ! queue ! h264parse ! queue ! %s ", pipeline);

	gst_init(argc, argv);
	src.timer = g_timer_new();
	src.loop = g_main_loop_new(NULL, TRUE);
	src.pipeline = gst_parse_launch(pipeline_str, &parse_err);

	if (src.pipeline == NULL) {
		if (parse_err != NULL) {
			THETA_LOG_ERROR("gst_src_init: pipeline parse failed: %s", parse_err->message);
			THETA_LOG_ERROR("gst_src_init: pipeline string: %s", pipeline_str);
			g_error_free(parse_err);
		} else {
			THETA_LOG_ERROR("gst_src_init: pipeline parse failed, pipeline is NULL (no error details)");
			THETA_LOG_ERROR("gst_src_init: pipeline string: %s", pipeline_str);
		}
		return FALSE;
	}
	gst_pipeline_set_clock(GST_PIPELINE(src.pipeline), gst_system_clock_obtain());

	src.appsrc = gst_bin_get_by_name(GST_BIN(src.pipeline), "ap");
	if (src.appsrc == NULL) {
		THETA_LOG_ERROR("gst_src_init: gst_bin_get_by_name('ap') failed, appsrc is NULL (check pipeline has element named 'ap')");
		return FALSE;
	}

	caps = gst_caps_new_simple("video/x-h264",
		"framerate", GST_TYPE_FRACTION, 30000, 1001,
		"stream-format", G_TYPE_STRING, "byte-stream",
		"profile", G_TYPE_STRING, "constrained-baseline", NULL);
	if (caps == NULL) {
		THETA_LOG_ERROR("gst_src_init: gst_caps_new_simple failed");
		return FALSE;
	}
	gst_app_src_set_caps(GST_APP_SRC(src.appsrc), caps);
	gst_caps_unref(caps);

	bus = gst_pipeline_get_bus(GST_PIPELINE(src.pipeline));
	src.bus_watch_id = gst_bus_add_watch(bus, gst_bus_cb, NULL);
	gst_object_unref(bus);
	if (src.bus_watch_id == 0) {
		THETA_LOG_ERROR("gst_src_init: gst_bus_add_watch failed (could not add bus watch)");
		return FALSE;
	}

	THETA_LOG_INFO("gst_src_init: pipeline created successfully");
	return TRUE;
}

void *
keywait(void *arg)
{
	struct gst_src *s;
	char keyin[4];
	ssize_t n;

	n = read(1, keyin, 1);
	if (n < 0) {
		THETA_LOG_WARNING("keywait: read from stdin failed, errno=%d (%s)", errno, strerror(errno));
	}
	if (n == 0) {
		THETA_LOG_INFO("keywait: stdin closed (EOF)");
	}

	s = (struct gst_src *)arg;
	THETA_LOG_INFO("keywait: user pressed key, stopping stream");
	g_main_loop_quit(s->loop);

	return NULL;
}

void
cb(uvc_frame_t *frame, void *ptr)
{
	struct gst_src *s;
	GstBuffer *buffer;
	GstFlowReturn ret;
	GstMapInfo map;
	gdouble ms;
	uint32_t pts;

	s = (struct gst_src *)ptr;
	ms = g_timer_elapsed(s->timer, NULL);

	buffer = gst_buffer_new_allocate(NULL, frame->data_bytes, NULL);
	if (buffer == NULL) {
		THETA_LOG_ERROR("cb: gst_buffer_new_allocate failed, data_bytes=%zu", (size_t)frame->data_bytes);
		return;
	}
	GST_BUFFER_PTS(buffer) = frame->sequence * s->dwFrameInterval*100;
	GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
	GST_BUFFER_DURATION(buffer) = s->dwFrameInterval*100;
	GST_BUFFER_OFFSET(buffer) = frame->sequence;
	s->framecount++;

	if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
		THETA_LOG_ERROR("cb: gst_buffer_map failed, frame=%u", s->framecount);
		gst_buffer_unref(buffer);
		return;
	}
	memcpy(map.data, frame->data, frame->data_bytes);
	gst_buffer_unmap(buffer, &map);

	g_signal_emit_by_name(s->appsrc, "push-buffer", buffer, &ret);
	gst_buffer_unref(buffer);

	if (ret != GST_FLOW_OK) {
		const char *flow_name = (ret == GST_FLOW_FLUSHING) ? "FLUSHING" :
			(ret == GST_FLOW_EOS) ? "EOS" :
			(ret == GST_FLOW_ERROR) ? "ERROR" :
			(ret == GST_FLOW_NOT_SUPPORTED) ? "NOT_SUPPORTED" : "UNKNOWN";
		THETA_LOG_WARNING("cb: push-buffer failed, ret=%d (%s), frame=%u", ret, flow_name, s->framecount);
	}
	return;
}

int
main(int argc, char **argv)
{
	uvc_context_t *ctx;
	uvc_device_t *dev;
	uvc_device_t **devlist;
	uvc_device_handle_t *devh;
	uvc_stream_ctrl_t ctrl;
	uvc_error_t res;

	pthread_t thr;
	pthread_attr_t attr;

	struct gst_src *s;
	int idx;
	char *pipe_proc;
	char *cmd_name;

	cmd_name = rindex(argv[0], '/');
	if (cmd_name == NULL)
		cmd_name = argv[0];
	else
		cmd_name++;

	if (strcmp(cmd_name, "gst_loopback") == 0) {
		pipe_proc = "decodebin ! autovideoconvert ! "
			"video/x-raw,format=I420 ! identity drop-allocation=true !"
			"v4l2sink device=/dev/video2 sync=false";
		THETA_LOG_INFO("main: starting as gst_loopback, pipeline=v4l2sink");
	} else {
		pipe_proc = " decodebin ! autovideosink sync=false";
		THETA_LOG_INFO("main: starting as gst_viewer, pipeline=autovideosink");
	}

	if (!gst_src_init(&argc, &argv, pipe_proc)) {
		THETA_LOG_ERROR("main: gst_src_init failed");
		THETA_LOG_INFO("main: exiting with -1 (gst_src_init failed)");
		return -1;
	}

	res = uvc_init(&ctx, NULL);
	if (res != UVC_SUCCESS) {
		THETA_LOG_ERROR("main: uvc_init failed, res=%d (%s)", res, uvc_strerror(res));
		uvc_perror(res, "uvc_init");
		THETA_LOG_INFO("main: exiting with res=%d (uvc_init failed)", res);
		return res;
	}
	THETA_LOG_INFO("main: uvc_init success");

	if (argc > 1 && strcmp("-l", argv[1]) == 0) {
		THETA_LOG_INFO("main: list mode (-l), enumerating THETA devices");

		res = thetauvc_find_devices(ctx, &devlist);
		if (res != UVC_SUCCESS) {
			THETA_LOG_ERROR("main: thetauvc_find_devices failed in list mode, res=%d (%s)", res, uvc_strerror(res));
			uvc_perror(res,"");
			uvc_exit(ctx);
			THETA_LOG_INFO("main: exiting with res=%d (list mode find_devices failed)", res);
			return res;
		}

		idx = 0;
		THETA_LOG_INFO("main: list mode, printing device list");
		printf("No : %-18s : %-10s\n", "Product", "Serial");
		while (devlist[idx] != NULL) {
			uvc_device_descriptor_t *desc;

			if (uvc_get_device_descriptor(devlist[idx], &desc) != UVC_SUCCESS) {
				THETA_LOG_WARNING("main: uvc_get_device_descriptor failed for device idx=%d, skipping", idx);
				idx++;
				continue;
			}

			printf("%2d : %-18s : %-10s\n", idx, desc->product,
				desc->serialNumber);

			uvc_free_device_descriptor(desc);
			idx++;
		}

		uvc_free_device_list(devlist, 1);
		uvc_exit(ctx);
		THETA_LOG_INFO("main: list mode completed, exiting");
		exit(0);
	}

	src.framecount = 0;
	THETA_LOG_INFO("main: finding THETA device index 0");
	res = thetauvc_find_device(ctx, &dev, 0);
	if (res != UVC_SUCCESS) {
		THETA_LOG_ERROR("main: THETA not found, thetauvc_find_device failed, res=%d (%s)", res, uvc_strerror(res));
		goto exit;
	}

	res = uvc_open(dev, &devh);
	if (res != UVC_SUCCESS) {
		THETA_LOG_ERROR("main: cannot open THETA device, uvc_open failed, res=%d (%s)", res, uvc_strerror(res));
		goto exit;
	}
	THETA_LOG_INFO("main: device opened, setting pipeline to PLAYING");

	{
		GstStateChangeReturn gst_res = gst_element_set_state(src.pipeline, GST_STATE_PLAYING);
		if (gst_res == GST_STATE_CHANGE_FAILURE) {
			THETA_LOG_ERROR("main: gst_element_set_state PLAYING failed (gst_res=%d)", gst_res);
			THETA_LOG_ERROR("main: check gst_bus_cb for pipeline error details, or /dev/video2 for gst_loopback");
			res = -1; /* non-UVC error */
			uvc_close(devh);
			goto exit;
		}
	}
	THETA_LOG_INFO("main: pipeline set to PLAYING successfully");

	if (pthread_create(&thr, NULL, keywait, &src) != 0) {
		THETA_LOG_ERROR("main: pthread_create(keywait) failed, errno=%d (%s)", errno, strerror(errno));
		uvc_close(devh);
		res = -1;
		goto exit;
	}
	THETA_LOG_INFO("main: keywait thread started");

	res = thetauvc_get_stream_ctrl_format_size(devh,
			THETAUVC_MODE_UHD_2997, &ctrl);
	if (res != UVC_SUCCESS) {
		THETA_LOG_ERROR("main: thetauvc_get_stream_ctrl_format_size failed, res=%d (%s)", res, uvc_strerror(res));
		uvc_close(devh);
		goto exit;
	}
	src.dwFrameInterval = ctrl.dwFrameInterval;
	src.dwClockFrequency = ctrl.dwClockFrequency;

	res = uvc_start_streaming(devh, &ctrl, cb, &src, 0);
	if (res != UVC_SUCCESS) {
		THETA_LOG_ERROR("main: uvc_start_streaming failed, res=%d (%s)", res, uvc_strerror(res));
		uvc_close(devh);
		goto exit;
	}

	THETA_LOG_INFO("main: streaming started, hit any key to stop");
	g_main_loop_run(src.loop);

	THETA_LOG_INFO("main: stopping streaming, cleaning up");
	uvc_stop_streaming(devh);
	THETA_LOG_INFO("main: uvc_stop_streaming done");

	{
		GstStateChangeReturn gst_res = gst_element_set_state(src.pipeline, GST_STATE_NULL);
		if (gst_res == GST_STATE_CHANGE_FAILURE) {
			THETA_LOG_WARNING("main: gst_element_set_state NULL failed (gst_res=%d)", gst_res);
		} else {
			THETA_LOG_INFO("main: pipeline set to NULL successfully");
		}
	}
	g_source_remove(src.bus_watch_id);
	g_main_loop_unref(src.loop);
	THETA_LOG_INFO("main: bus watch and loop cleaned up");

	pthread_cancel(thr);
	pthread_join(thr, NULL);
	THETA_LOG_INFO("main: keywait thread joined");

	uvc_close(devh);
	THETA_LOG_INFO("main: streaming completed successfully");

exit:
	uvc_exit(ctx);
	THETA_LOG_INFO("main: exiting, res=%d", res);
	return res;
}
