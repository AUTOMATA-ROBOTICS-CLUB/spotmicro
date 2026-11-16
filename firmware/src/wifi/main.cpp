// Linux client: reads a game controller from /dev/input and sends every button
// press to the robot over UDP as a 4-byte big-endian evdev key code (see
// wifi.h on the ESP32 side).
//
// Build: g++ -std=c++17 -Wall -o controller main.cpp
// Run:   join the robot's WiFi AP, then ./controller (needs read access to
//        /dev/input/event*: run as root or add yourself to the input group)

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/input.h>
#include <map>
#include <netinet/in.h>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#define IS_DUALSENSE 1
#define IS_XBOX 0
// ESP32 UDP server port
#define PORT 4210
// ESP32 WiFi access point IP
#define IP "192.168.4.1"

#if IS_DUALSENSE
#define PATH "Sony Interactive Entertainment DualSense Wireless Controller"
#endif

// if this is one set currentPress to the correct event key and send over udp
std::atomic<int> buttonIsPressed{0};
std::atomic<int> currentPress{0};

// TODO: Add 2 seperate threads, one for udp client and one for processing
// inputs

std::map<int, std::string_view> btnMap = {
    {304, "Cross"}, {305, "Circle"}, {307, "Triangle"}, {308, "Square"}};

void getEvent(int event) {
  if (auto it = btnMap.find(event); it != btnMap.end()) {
    std::cout << it->second << "\n";
  }
}

// Returns the N of the /dev/input/eventN whose name is PATH, or -1.
int FindDevice(void) {
  std::cout << "looking for device"
            << "\n";

  for (int i = 0; i < 64; i++) {
    char path[64];
    snprintf(path, sizeof(path), "/dev/input/event%d", i);
    int fd = open(path, O_RDONLY);
    if (fd < 0)
      continue;

    char name[256] = {};
    bool found = ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 &&
                 strcmp(name, PATH) == 0;
    close(fd);

    if (found) {
      std::cout << "Found " << path << "\n";
      return i;
    }
  }

  std::cout << "Device was not found (is it connected, and can you read "
               "/dev/input?)"
            << "\n";
  return -1;
}

// Opens the controller for blocking reads, so the loops sleep until there is
// an event. Returns the file descriptor, or -1.
int openController(void) {
  int i = FindDevice();
  if (i < 0)
    return -1;

  char path[64];
  snprintf(path, sizeof(path), "/dev/input/event%d", i);
  int fd = open(path, O_RDONLY);
  if (fd < 0)
    std::cout << "Error opening " << path << ": " << strerror(errno) << "\n";
  return fd;
}

// Create UDP Client for esp32 wifi connection
int createClient(void) {
  int fd = openController();
  if (fd < 0)
    return 1;

  int s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) {
    std::cout << "Could not create socket: " << strerror(errno) << "\n";
    close(fd);
    return 1;
  }

  struct sockaddr_in server_addr = {};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  if (inet_pton(AF_INET, IP, &server_addr.sin_addr) <= 0) {
    std::cout << "Wrong address"
              << "\n";
    close(s);
    close(fd);
    return 1;
  }

  struct input_event event;
  while (true) {
    ssize_t n = read(fd, &event, sizeof(event));
    // The controller was unplugged or turned off.
    if (n < 0 && errno != EINTR) {
      std::cout << "Controller disconnected: " << strerror(errno) << "\n";
      break;
    }
    if (n != sizeof(event))
      continue;

    if (event.type == EV_KEY && event.value == 1) {
      uint32_t packet = htonl(event.code);
      getEvent(event.code);
      std::cout << "Event Code: " << event.code << "\n";
      if (sendto(s, &packet, sizeof(packet), 0, (struct sockaddr *)&server_addr,
                 sizeof(server_addr)) < 0)
        std::cout << "Send failed (joined the robot's WiFi?): "
                  << strerror(errno) << "\n";
    }

    else if (event.type == EV_ABS) {
      if (event.code == ABS_HAT0X) {
        if (event.value == 1) {
          std::cout << "Right"
                    << "\n";
        } else if (event.value == -1) {
          std::cout << "Left"
                    << "\n";
        }
      } else if (event.code == ABS_HAT0Y) {
        if (event.value == 1) {
          std::cout << "Down"
                    << "\n";
        } else if (event.value == -1) {
          std::cout << "Up"
                    << "\n";
        }
      }
    }
  }

  close(fd);
  close(s);
  return 1;
}

// Keep until i add different threads for the udp client and this function
void controller_loop(void) {
  int fd = openController();
  if (fd < 0)
    return;

  struct input_event event;
  while (read(fd, &event, sizeof(event)) == sizeof(event)) {
    if (event.type == EV_KEY) {
      getEvent(event.code);
    }
    // else if(event.type == EV_ABS) {
    // std::cout << "Joystick event" << "\n";
    //}
  }
  close(fd);
}

int main(void) { return createClient(); }
