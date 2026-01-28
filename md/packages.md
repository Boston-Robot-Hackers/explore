## The Camera Stack on Raspberry Pi + Ubuntu
### Pito Salas with assistance from two humans and two AIs.

## Intro

THere are two challenges in writing this down. First it is very complciated and intertwined. Ans second, it chantes every time a new release comes out. This is my best cut. I don't promise total accuracy but hopefully this is useful to you.

### Background

Like many architecures, we can undetrstand it better if we see how the pieces fit together in a rough layer cake. The complications come from the fact that different bits have quite different histories. Ubuntu as an OS has support for cameras. Rasberry Pi as a hardware platform has some of that too. But Rasberry Pi's camera support is more related to the Rasberry Pi OS. So some code that comes from Rasberry Pi OS needs to be recompiled on Ubuntu with differrent tools and build processes.

 ### Hardware and Firmware

At the lowest level, the Raspberry Pi has a dedicated **Image Signal Processor (ISP)**. The ISP handles raw sensor data processing—demosaicing, white balance, auto-exposure, and noise reduction. Libraries that interface with this hardware translate between raw sensor output and usable image data.

### libcamera (The Core Library)

`libcamera` is a modern Linux camera stack that builds on top of the older V4L2-based approaches and is designed to handle complex camera pipelines. It provides. it is designed to 

- Pipeline handling for the ISP
- A unified, device-agnostic camera API
- Control of sensor + ISP pipelines
- Metadata-driven control (exposure, gain, focus, etc.)
- Integration with—but not replacement of—V4L2 at the kernel level

Note that "modern linux" encompasses various distributions of Linux, there are numerous cameras and sensors, And, raspberry Pi maintains their own fork of libcamera with Pi-specific optimizations and tuning files for their camera sensors. 

### Libcamera-apps / rpicam-apps

These are command-line applications built on libcamera, specifically for Rasberry Pi

- `libcamera-hello` / `rpicam-hello` — camera preview
- `libcamera-still` / `rpicam-still` — capture stills
- `libcamera-vid` / `rpicam-vid` — capture video

Raspberry Pi renamed these tools from `libcamera-*` to `rpicam-*` (late 2023) to clarify that they are Rasberry Pi specific tools built on top of libcamera. 

### Layer 4: ROS2 Camera Nodes

Packages like `camera-ros` sit on top of libcamera and publish camera data to ROS2 topics. Now we get into how linux apps are compiled and linked. At compile time, the camera-ros is built against an API version and that and the name of the shared library is recorded in the binary. The actual/fitting library is located at run-time.

This issue is a special case. In general when you install something with sudo apt install xxx you are getting binaries that were built in a certain context. If you change the context you may have to rebuild the package and have it "override" the one installed by apt install. In the ROs world, you can do that by cloning the repo for the package in your own ros2_ws and then colcon build will build it locally with the up to date context. 

---

## Linux Utilities for Library Management

### ldd — List Dynamic Dependencies

Shows which shared libraries (`.so` files) an executable needs at runtime:
```bash
ldd /usr/bin/rpicam-hello
```
Useful for debugging "library not found" errors or version mismatches. `ldd` shows the dependency tree based on what is recorded in the ELF file header[s]. But some programs do their own dynamic loading as well and ldd would not be able to help with that:-( I believe around libcamera, the libpisp might be loading specific parts for different cameras.


### ldconfig — Update Library Cache

Linux caches the locations of shared libraries in `/etc/ld.so.cache`. After installing libraries from source, run:
```bash
sudo ldconfig
```
This refreshes the cache so programs can find the new libraries.  Normally, you should not have to do that. Well built packages or 'make-files' should re-run that command on 'install' when needed.
---

## The Binary Compatibility Problem

When you install ROS2 packages via apt (`apt install ros-jazzy-camera-ros`), you get pre-compiled binaries. These binaries were compiled against whatever API version and library name of libcamera existed when the package was built.

**The problem:** If you later compile and install a newer libcamera from source, the apt-installed ROS packages may be incompatible—they expect the old library's symbols and behavior. That's true in the most general case. But the shared library system with it's naming conventions and recording of required API versions should prevent most surprises.

What you more frequently run into are search/path issues between the system libraries and your custom built ones.

The 'ros-jazzy-camera-ros' package depends on the 'ros-jazzy-libcamera' package, so all those dependencies will be installed (and potentially) updated. On the RasPi, there is also the system 'libcamera' package, which adds more version confusion:-(

When you build libcamera from source, by default, it would install into /usr/local/..., adding potentially a third version to the mix. You could bend the environment to find your versions, but that's usually error prone in the long run.

You could force the build to override one of the packaged versions, but that's not a good idea. You should never override 'package-managed' locations with 'manual-build' things. The next update of that package will break things!

**The solution:** Clone the ROS package source into your workspace (`ros2_ws/src/`) and rebuild with `colcon build`. This compiles the package against your current libcamera installation, ensuring compatibility.

That will most likely work but it gets cumbersome when you have to do this for a couple of packages. And now you have 'one more' package to keep up-to-date.

That's why I started building those libcamera and related packages and to publish them. They follow all the normal packaging standards and just replace the 'Ubuntu-company-supplied' packages with newer versions. The versioning makes sure, only packages with support for the latest PiCam's get updated;-)

My packages also satisfy the 'ros-jazzy-libcamera' dependency with a newer version. So, the normal 'ros-jazzy-camera-ros' package should be happy.

In the end, with my packages, you should have one libcamera.so and everything should use it;-)
