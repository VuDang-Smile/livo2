# 
#   Copyright 2020 K. Takeo. All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   1. Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
#   3. Neither the name of the author nor other contributors may be
#   used to endorse or promote products derived from this software
#   without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 


PKG_CONFIG_LIB = gstreamer-app-1.0 libuvc

CFLAGS += $(shell pkg-config --cflags $(PKG_CONFIG_LIB))
LDFLAGS += $(shell pkg-config --libs $(PKG_CONFIG_LIB))

SRCS = gst_viewer.c thetauvc.c
OBJS := $(SRCS:.c=.o)
EXES = gst_viewer gst_loopback

all: $(EXES)

gst_viewer: $(OBJS)
	$(CC) $(OBJS) -o $@  $(LDFLAGS) -pthread

gst_loopback: gst_viewer
	ln -sf $< $@

test_theta: test_theta.c thetauvc.c
	$(CC) test_theta.c thetauvc.c -o $@ -I/usr/local/include -L/usr/local/lib -luvc -lusb-1.0

# Video device setup
setup-video-device:
	@echo "Tạo /dev/video2 cho loopback streaming..."
	sudo ./create_video2.sh

# QR Code Detection targets
install-deps:
	@echo "Cài đặt Python dependencies cho QR detection..."
	pip3 install opencv-python numpy pyzbar

qr-detect: install-deps
	@echo "Chạy QR detection cơ bản với Theta X..."
	sudo ./run_theta_qr.sh

qr-detect-enhanced: install-deps
	@echo "Chạy Enhanced QR Detection (khuyến nghị)..."
	sudo ./start_enhanced.sh

qr-detect-no-display: install-deps
	@echo "Chạy QR detection (không hiển thị video)..."
	sudo python3 qr_detector.py --video /dev/video2 --no-display

enhanced-no-display: install-deps
	@echo "Chạy Enhanced QR Detection (không hiển thị video)..."
	sudo python3 qr_detector_enhanced.py --video /dev/video2 --no-display

list-cameras:
	@echo "Liệt kê các camera có sẵn:"
	./gst_loopback -l

test-camera:
	@echo "Test camera Theta X..."
	sudo ./gst_loopback

clean:
	@$(RM) $(OBJS) $(EXES)
	@$(RM) -f /tmp/theta_qr_running
